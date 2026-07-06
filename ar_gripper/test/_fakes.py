"""Hardware-free stand-ins for the Feetech serial bus.

Nothing here talks to real hardware. ``FakeServo`` models one Feetech SMS/STS
servo's register file and the half-duplex packet protocol exactly as
``ar_gripper.feetech`` speaks it, so the *real* ``feetech``/``gripper`` code runs
unmodified against an in-memory stand-in. ``FakeSerial`` is the drop-in for the
object ``serial.serial_for_url`` returns; it records every instruction packet the
driver writes (``trace``) so tests can assert byte-identical wire behaviour.

These live in a plain module (not ``conftest``) so both the pytest fixtures and
the offline golden-capture step can import them.
"""


def checksum(values):
    """Feetech checksum: bitwise-NOT of the byte sum, mod 256 (mirrors feetech.py)."""
    return (~sum(values)) % 256


class FakeServo:
    """In-memory register file for one Feetech servo.

    Register bytes are stored little-endian to match the driver's getters
    (``data[0] | data[1] << 8``). A handful of "live" registers are computed so
    the state machines in ``gripper.py`` (calibrate / goto) actually converge:

    * ``present_position`` (0x38) mirrors the last commanded ``goal_position``
      (0x2A); a ``reset_current_position`` write (0x28 == 128) snaps it to 2048.
      This models the servo instantly reaching its target so ``_wait_for_stop``
      settles deterministically.
    * ``present_load`` (0x3C) yields ``load_sequence`` values in order (repeating
      the last once exhausted) so calibration's load-based loops terminate.
    * ``present_current`` (0x45) yields ``current_value`` (constant by default).
    * ``moving_sign`` (0x42) yields ``moving`` (0 == stopped by default).
    """

    GOAL_POSITION = 0x2A
    TORQUE_SWITCH = 0x28
    PRESENT_POSITION = 0x38
    PRESENT_LOAD = 0x3C
    PRESENT_CURRENT = 0x45
    MOVING_SIGN = 0x42
    RESET_MIDPOINT = 128

    def __init__(self):
        self.reg = bytearray(0x60)  # addresses 0x00..0x5F
        self.present_position_value = 150  # decoded ticks
        self.load_sequence = [0.0]
        self._load_idx = 0
        self.current_value = 0.0
        self.moving = 0
        self.temperature = 25
        self.voltage_raw = 120  # -> present_voltage 12.0V (read off 0x3A)
        self._apply_init_defaults()

    def _apply_init_defaults(self):
        # Values chosen so Gripper._init_servo's _set_if_different comparisons all
        # match (servo already configured) -> no EEPROM writes, deterministic trace.
        self._set_word(0x18, 50)  # minimum_startup_force raw*0.1 == 5.0
        self._set_word(0x10, 1000)  # max_torque raw*0.1 == 100.0
        self.reg[0x24] = 30  # overload_torque == 30
        self.reg[0x22] = 20  # protection_torque == 20
        self.reg[0x23] = 30  # protection_time raw*10 == 300
        self.reg[0x21] = 0  # drive_mode == 0
        self.reg[0x3A] = self.voltage_raw  # present_voltage reads 0x3A (per feetech.py)
        self.reg[0x3F] = self.temperature & 0xFF  # present_temperature

    def _set_word(self, addr, value):
        self.reg[addr] = value & 0xFF
        self.reg[addr + 1] = (value >> 8) & 0xFF

    def _encode_position(self, value):
        sign = 1 if value < -1 else 0
        return bytes([value & 0xFF, (value >> 8) & 0x7F | sign << 7])

    def read(self, addr, n):
        if addr == self.PRESENT_POSITION:
            return self._encode_position(self.present_position_value)[:n]
        if addr == self.PRESENT_LOAD:
            raw = int(round(self._next_load() * 10))
            return bytes([raw & 0xFF, (raw >> 8) & 0xFF])[:n]
        if addr == self.PRESENT_CURRENT:
            raw = int(round(self.current_value / 6.5))
            return bytes([raw & 0xFF, (raw >> 8) & 0xFF])[:n]
        if addr == self.MOVING_SIGN:
            return bytes([self.moving & 0xFF])[:n]
        return bytes(self.reg[addr : addr + n])

    def write(self, addr, data):
        if (
            addr == self.TORQUE_SWITCH
            and len(data) == 1
            and data[0] == self.RESET_MIDPOINT
        ):
            # Overloaded: writing 128 resets the current position to 2048.
            self.present_position_value = 2048
            return
        self.reg[addr : addr + len(data)] = bytes(data)
        if addr == self.GOAL_POSITION and len(data) >= 2:
            self.present_position_value = data[0] | (data[1] & 0x7F) << 8
            if data[1] & 0x80:
                self.present_position_value *= -1

    def _next_load(self):
        value = self.load_sequence[min(self._load_idx, len(self.load_sequence) - 1)]
        self._load_idx += 1
        return value


class FakeSerial:
    """Drop-in for the object ``serial.serial_for_url`` returns.

    Half-duplex: each ``write`` of a full instruction packet immediately buffers
    the servo's reply for the following ``read`` calls, exactly as the real bus
    round-trips. Every written packet is appended to ``trace`` for byte-level
    assertions.
    """

    def __init__(self):
        self.servos = {}
        self._out = bytearray()
        self.trace = []
        self.timeout = 0.2
        self.url = None

    def servo(self, servo_id):
        return self.servos.setdefault(servo_id, FakeServo())

    # -- pyserial-compatible surface -------------------------------------------------
    def write(self, data):
        data = bytes(data)
        self.trace.append(data)
        self._out += self._reply_for(data)
        return len(data)

    def read(self, n=1):
        chunk = bytes(self._out[:n])
        del self._out[:n]
        return chunk

    def flushInput(self):
        self._out.clear()

    def flushOutput(self):
        pass

    def reset_input_buffer(self):
        self._out.clear()

    def close(self):
        pass

    # -- protocol ---------------------------------------------------------------------
    def _reply_for(self, packet):
        # FF FF <id> <len> <instruction...> <checksum>; instruction is (len-1) bytes.
        if len(packet) < 6 or packet[0] != 0xFF or packet[1] != 0xFF:
            return b""
        servo_id = packet[2]
        length = packet[3]
        instruction = packet[4 : 4 + (length - 1)]
        instr = instruction[0]
        servo = self.servo(servo_id)
        if instr == 0x02:  # read
            addr, n = instruction[1], instruction[2]
            payload = servo.read(addr, n)
        elif instr == 0x03:  # write
            addr, wdata = instruction[1], instruction[2:]
            servo.write(addr, wdata)
            payload = b""
        else:
            return b""
        body = [servo_id, len(payload) + 2, 0] + list(payload)
        return bytes([0xFF, 0xFF] + body + [checksum(body)])


class FakeTime:
    """Monotonic fake clock: each ``time()`` advances by ``step``; ``sleep`` no-ops.

    Guarantees the wait/servo loops in gripper.py make progress and terminate
    without real wall-clock delay, keeping traces deterministic.
    """

    def __init__(self, step=0.05):
        self._t = 0.0
        self.step = step

    def time(self):
        value = self._t
        self._t += self.step
        return value

    def sleep(self, _seconds):
        self._t += self.step


def install_fake_serial(monkeypatch_or_none=None):
    """Patch ``serial.serial_for_url`` to hand out FakeSerials.

    Returns ``(created, restore)`` where ``created`` is the growing list of
    FakeSerials (construction order) and ``restore`` puts the real function back.
    Usable both from pytest (pass a monkeypatch) and offline capture (pass None).
    """
    from ar_gripper import feetech

    created = []
    original = feetech.serial.serial_for_url

    def factory(url=None, **_kwargs):
        fake = FakeSerial()
        fake.url = url
        created.append(fake)
        return fake

    if monkeypatch_or_none is not None:
        monkeypatch_or_none.setattr(feetech.serial, "serial_for_url", factory)
        return created, lambda: None

    feetech.serial.serial_for_url = factory
    return created, lambda: setattr(feetech.serial, "serial_for_url", original)
