"""In-memory Feetech bus for hardware-free testing (loopback mock).

Nothing here talks to real hardware. ``FakeServo`` models one Feetech SMS/STS
servo's register file and the half-duplex packet protocol exactly as
``ar_gripper.feetech`` speaks it, so the *real* ``feetech`` / ``gripper`` /
``standalone`` code runs unmodified against an in-memory stand-in. ``FakeSerial``
is the drop-in for the object ``serial.serial_for_url`` returns; it records every
instruction packet the driver writes (``trace``) for byte-level assertions.

This module is ROS-free (only ``pyserial`` + the standard library) and ships in
the installed package, so an external process — e.g. a LeRobot pipeline that
consumes ``ARGripperStandalone`` — can loopback-test its gripper integration with
no hardware and no ROS:

    from ar_gripper.mock import mock_gripper

    with mock_gripper() as (gripper, bus):
        gripper.close()
        assert gripper.get_position("ticks") > 4000     # closed
        gripper.open()
        assert gripper.get_position("ticks") < 200       # open
        # bus[-1].trace holds every servo packet that was written

For a fresh homing/calibration run instead of the default "reuse a saved
position" path, pass ``saved_position=None`` with a ``load_sequence`` that lets
the homing loop find contact, e.g. ``mock_gripper(saved_position=None,
load_sequence=[5, 15, 15, 0])``.

Lower-level building blocks (``FakeServo``, ``FakeSerial``, ``FakeTime``,
``install_fake_serial``, ``loopback_bus``) are exposed for finer control.
"""

from contextlib import contextmanager

__all__ = [
    "FakeServo",
    "FakeSerial",
    "FakeTime",
    "HOMING_LOAD_SEQUENCE",
    "checksum",
    "install_fake_serial",
    "install_ros_node_loopback",
    "loopback_bus",
    "mock_gripper",
]

# present_load reads that let Gripper.calibrate()'s homing loop find contact
# (>= 10) and then retreat until there is no load (0), so a fresh mock startup
# rehomes deterministically through the *real* calibration routine (mirrors the
# mock_gripper(saved_position=None, load_sequence=...) example above).
HOMING_LOAD_SEQUENCE = (5, 15, 15, 0)


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
    Usable both from pytest (pass a monkeypatch) and offline (pass None).
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


def install_ros_node_loopback(fast_clock=True):
    """Route a long-running ARGripper ROS node onto an in-memory FakeServo bus.

    Like ``loopback_bus`` but for a *persistent* process (the ROS node stays
    mocked for its whole lifetime rather than for a ``with`` block): patches
    ``serial.serial_for_url`` so every ``USB2FeetechDevice`` opens a
    ``FakeSerial``, and -- with ``fast_clock`` -- swaps ``gripper.py``'s wait-loop
    clock for ``FakeTime`` so grasp/homing loops make progress without real delay
    (and without busy-spinning the node's executor through the inrush wait). This
    is what lets ``ros2 launch ar_gripper ar_gripper_control.launch.py mock:=true``
    run the *real* driver / action / calibration / diagnostics code with no
    hardware. ROS-free (does not import rclpy).

    Returns ``(created, restore)``: ``created`` is the growing list of FakeSerials
    (one per device opened; ``created[-1]`` is the newest, whose ``servo(id)`` the
    caller seeds -- e.g. with ``HOMING_LOAD_SEQUENCE`` -- before the gripper
    calibrates). ``restore`` undoes both patches; the node discards it (staying
    mocked), tests keep it for teardown.
    """
    created, restore_serial = install_fake_serial(None)
    if not fast_clock:
        return created, restore_serial

    from ar_gripper import gripper as _gripper

    original_time = _gripper.time
    _gripper.time = FakeTime()

    def restore():
        restore_serial()
        _gripper.time = original_time

    return created, restore


@contextmanager
def loopback_bus(fast_clock=True):
    """Context manager: route ``USB2FeetechDevice`` onto an in-memory FakeServo bus.

    Yields the growing list of ``FakeSerial`` objects (one per device opened while
    active); ``bus[-1]`` is the most recently opened. With ``fast_clock`` (default)
    the gripper's wait/servo loops run instantly and deterministically. Restores
    the real serial factory (and clock) on exit.
    """
    created, restore = install_fake_serial(None)
    original_time = None
    if fast_clock:
        from ar_gripper import gripper as _gripper

        original_time = _gripper.time
        _gripper.time = FakeTime()
    try:
        yield created
    finally:
        restore()
        if fast_clock:
            from ar_gripper import gripper as _gripper

            _gripper.time = original_time


@contextmanager
def mock_gripper(
    servo_id=0,
    name="primary",
    present_position=150,
    saved_position=150,
    load_sequence=None,
    servo_position_path=None,
    fast_clock=True,
):
    """Context manager yielding ``(ARGripperStandalone, FakeSerial)`` on a mock bus.

    The default reuses a saved calibration matching the servo's position (no
    physical homing), so the gripper comes up calibrated and ready for
    ``open()`` / ``close()`` / ``set_goal()``. To exercise a fresh homing run,
    pass ``saved_position=None`` and a ``load_sequence`` the homing loop can find
    contact in (e.g. ``[5, 15, 15, 0]``). If ``servo_position_path`` is None a
    temporary file is used and cleaned up.
    """
    import json
    import os
    import tempfile

    from ar_gripper.feetech import USB2FeetechDevice
    from ar_gripper.standalone import ARGripperStandalone

    tmp_path = None
    if servo_position_path is None:
        fd, servo_position_path = tempfile.mkstemp(
            prefix="ar_gripper_mock_", suffix=".json"
        )
        os.close(fd)
        tmp_path = servo_position_path
        if saved_position is None:
            os.remove(servo_position_path)  # absent file -> forces a rehome
    if saved_position is not None:
        with open(servo_position_path, "w") as f:
            json.dump({"position": saved_position}, f)

    with loopback_bus(fast_clock=fast_clock) as bus:
        device = USB2FeetechDevice("loopback://")
        servo = bus[-1].servo(servo_id)
        servo.present_position_value = present_position
        if load_sequence is not None:
            servo.load_sequence = list(load_sequence)
        gripper = ARGripperStandalone(
            servo_id=servo_id,
            name=name,
            device=device,
            servo_position_path=servo_position_path,
        )
        try:
            yield gripper, bus[-1]
        finally:
            if tmp_path is not None and os.path.exists(tmp_path):
                os.remove(tmp_path)
