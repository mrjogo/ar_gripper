"""What ``IsaacServo`` adds on top of ``mock.FakeServo``.

The base class's own tests already cover the inherited physics (current
synthesis, the 6.5 mA/LSB quantisation, the exactly-zero unloaded load, the
"five baseline reads are five distinct samples" guard, stall-vs-free-close
discrimination) over the base class, and they apply here by inheritance -- not
repeated. This file only tests what ``IsaacServo`` itself introduces: that a
live read pulls exactly one Isaac sample, that the sample is the source for
position/load, that ticks and metres move in opposite directions, that a
rezero's tick offset survives, that a goal position becomes a metre command
without the fake's "instantly reaches target" leaking through, and that
``IsaacJointBus.set_drive_parameter`` undoes the wire's steps/s // 50 encoding.

No rclpy, no Isaac: driven by a fake ``IsaacJointBus`` whose ``wait_for_sample``
pops scripted ``(position_m, velocity_mps, effort_N)`` tuples and whose
``command`` / ``set_drive_parameter`` just record.
"""

import threading

import pytest

from ar_gripper.scripts.isaac_servo import _DRIVE_SPEED_ADDR, IsaacJointBus, IsaacServo

SID = 1


class _ScriptedIsaacJointBus:
    """Records ``command()`` / ``set_drive_parameter()`` calls.

    ``wait_for_sample()`` pops scripted ``(position_m, velocity_mps,
    effort_N)`` tuples in order, repeating the last one once exhausted -- so a
    test that only cares about one steady state can script a single tuple and
    read the servo as many times as it likes.
    """

    def __init__(self, samples):
        self._samples = list(samples)
        self.wait_for_sample_calls = 0
        self.commands = []
        self.drive_params = []

    def wait_for_sample(self, timeout_s=None):
        self.wait_for_sample_calls += 1
        if len(self._samples) > 1:
            return self._samples.pop(0)
        return self._samples[0]

    def command(self, position_m):
        self.commands.append(position_m)

    def set_drive_parameter(self, addr, value):
        self.drive_params.append((addr, value))


@pytest.fixture
def make_isaac_servo(fake_serials):
    """Build a calibrated ``Gripper`` over an ``IsaacServo`` on a scripted bus.

    Returns ``(gripper, bus, isaac_servo)`` -- ``gripper.servo`` is the
    ``FeetechSMSServo`` wrapper the driver talks to; ``isaac_servo`` is the
    underlying ``IsaacServo`` register file, for asserting on internals like
    ``_tick_offset`` that have no wire-visible getter. Marked calibrated so
    tests can drive ``goal_position`` / read ``present_position`` etc.
    directly without a physical homing run -- homing itself is exercised
    separately below.
    """
    from ar_gripper.feetech import USB2FeetechDevice
    from ar_gripper.gripper import Gripper

    def _make(samples):
        device = USB2FeetechDevice("/dev/fake")
        fake = fake_serials[-1]
        bus = _ScriptedIsaacJointBus(samples)
        isaac_servo = IsaacServo(bus)
        fake.servos[SID] = isaac_servo
        gripper = Gripper(device, "primary", SID)
        gripper._calibrated = True
        return gripper, bus, isaac_servo

    return _make


def test_position_reads_the_scripted_sample(make_isaac_servo):
    gripper, _bus, _isaac = make_isaac_servo([(0.02, 0.0, 0.0)])
    assert gripper.servo.present_position == round(
        IsaacServo.CLOSED_POSITION_TICKS - 0.02 * IsaacServo.TICKS_PER_METRE
    )
    assert gripper.servo.present_position == 2517


def test_closed_extreme_reads_4095_ticks(make_isaac_servo):
    """tick 4095 == 0.0 m == CLOSED (Gripper._POSITION_MAX, FINGER_CLOSED_POS).

    Ticks go DOWN as metres go UP, not up -- a sign a first draft of this
    file had backwards, live-verified against a running simulator.
    """
    gripper, _bus, _isaac = make_isaac_servo([(0.0, 0.0, 0.0)])
    assert gripper.servo.present_position == 4095


def test_open_extreme_reads_150_ticks(make_isaac_servo):
    """tick 150 == 0.05 m == OPEN (Gripper._POSITION_MIN, FINGER_OPEN_POS)."""
    gripper, _bus, _isaac = make_isaac_servo([(0.05, 0.0, 0.0)])
    assert gripper.servo.present_position == 150


def test_load_is_exactly_zero_when_unloaded(make_isaac_servo):
    gripper, _bus, _isaac = make_isaac_servo([(0.0, 0.0, 0.0)])
    assert gripper.servo.present_load == 0
    assert isinstance(gripper.servo.present_load, float)


def test_load_scales_with_effort(make_isaac_servo):
    half_stall = IsaacServo.MAX_EFFORT_N / 2  # 520.625 N
    gripper, _bus, _isaac = make_isaac_servo([(0.0, 0.0, half_stall)])
    assert gripper.servo.present_load == pytest.approx(50.0)


def test_one_live_read_consumes_exactly_one_sample(make_isaac_servo):
    """Static registers (temperature, voltage) must not block on Isaac.

    This is what keeps diagnostics alive during a grasp: they read
    ``present_temperature`` / ``present_voltage`` on the same servo object the
    stall loop busy-polls, and those fall straight through to the base
    class's in-memory register file.
    """
    gripper, bus, _isaac = make_isaac_servo([(0.0, 0.0, 0.0)])

    _ = gripper.servo.present_position
    _ = gripper.servo.present_load
    _ = gripper.servo.present_current
    _ = gripper.servo.moving_sign
    assert bus.wait_for_sample_calls == 4

    _ = gripper.servo.present_temperature
    _ = gripper.servo.present_voltage
    assert bus.wait_for_sample_calls == 4


def test_tick_offset_survives_a_rezero(make_isaac_servo):
    gripper, _bus, _isaac = make_isaac_servo(
        [(0.02, 0.0, 0.0), (0.02, 0.0, 0.0), (0.021, 0.0, 0.0)]
    )

    _ = gripper.servo.present_position  # prime a sample so the rezero below has one
    gripper.servo.reset_current_position()
    assert gripper.servo.present_position == 2048

    # Moving the (scripted) joint by +0.001 m (opening) moves the reported
    # position by -round(0.001 * TICKS_PER_METRE) == -79 ticks from the
    # rezeroed origin: ticks go DOWN as metres go UP.
    assert gripper.servo.present_position == 2048 - 79


def test_goal_position_becomes_a_metre_command(make_isaac_servo):
    gripper, bus, isaac_servo = make_isaac_servo([(0.02, 0.0, 0.0)])
    _ = gripper.servo.present_position  # prime a sample so the rezero below has one
    gripper.servo.reset_current_position()  # tick_offset set so 0.02 m reads 2048
    tick_offset = isaac_servo._tick_offset

    gripper.servo.goal_position = 4095

    assert bus.commands == [
        pytest.approx(
            (IsaacServo.CLOSED_POSITION_TICKS + tick_offset - 4095)
            / IsaacServo.TICKS_PER_METRE
        )
    ]
    assert len(bus.commands) == 1
    # The fake's "instantly reaches target" write-path must not leak through a
    # live read: the next present_position read pulls a fresh Isaac sample
    # (still 0.02 m -> 2048 ticks after the rezero above), not 4095.
    assert gripper.servo.present_position == 2048


class _HardStopIsaacBus:
    """Chases the last commanded goal at a fixed rate, clamping at a hard stop
    and reporting a stall force while pinned against it.

    The metres-and-Isaac-samples analogue of ``FakeServo``'s own
    ``obstruction_ticks`` model (see ``test_mock_motor_current.py``), used
    only by the homing test below: it reacts to whatever ``command()`` sends
    rather than following a fixed script, so it converges regardless of the
    exact tick<->metre offsets the calibration sequence works out along the
    way. The stop is a LOWER bound on position -- the finger's real closed
    limit sits at 0.0 m, and "closed" commands ever-decreasing metres (ticks
    go DOWN as metres go UP), so unlike ``FakeServo``'s own ticks-only
    obstruction (an upper bound, since ticks and "closing" increase together
    there) this one has to clamp from below.
    """

    STEP_M = 0.001  # metres advanced per wait_for_sample() call
    STALL_FORCE_N = 150.0  # -> present_load ~14.4, above the homing threshold of 10

    def __init__(self, closed_stop_m):
        self._closed_stop_m = closed_stop_m
        # Start well clear of the stop, same as FakeServo's own default
        # present_position_value not already being at its obstruction.
        self._position_m = closed_stop_m + 0.03
        self._goal_m = self._position_m
        self.wait_for_sample_calls = 0

    def wait_for_sample(self, timeout_s=None):
        self.wait_for_sample_calls += 1
        target = max(self._goal_m, self._closed_stop_m)
        delta = target - self._position_m
        if abs(delta) <= self.STEP_M:
            moved = delta
            self._position_m = target  # snap exactly, not via += (float noise)
        else:
            moved = self.STEP_M if delta > 0 else -self.STEP_M
            self._position_m += moved
        blocked = (
            self._goal_m < self._closed_stop_m
            and self._position_m == self._closed_stop_m
        )
        effort = self.STALL_FORCE_N if blocked else 0.0
        velocity = moved * 100.0  # nonzero while stepping, exactly 0.0 when settled
        return self._position_m, velocity, effort

    def command(self, position_m):
        self._goal_m = position_m

    def set_drive_parameter(self, addr, value):
        pass


def test_isaac_backed_calibration_converges(fake_serials, fast_clock):
    """Same shape as test_mock_motor_current.py's
    test_calibrate_homes_against_an_obstruction, driven from Isaac-shaped
    samples rather than the internal obstruction model.
    """
    from ar_gripper.feetech import USB2FeetechDevice
    from ar_gripper.gripper import Gripper

    device = USB2FeetechDevice("/dev/fake")
    fake = fake_serials[-1]
    fake.servos[SID] = IsaacServo(_HardStopIsaacBus(closed_stop_m=0.02))
    gripper = Gripper(device, "primary", SID)

    assert gripper.calibrate() is True
    assert gripper.calibrated is True


def test_drive_speed_conversion_undoes_the_wire_50x_encoding():
    """A live simulator run caught this: IsaacJointBus.set_drive_parameter
    must scale the raw DRIVE_SPEED register word back up by 50 before treating
    it as metres/s, because the wire encodes steps/s // 50
    (FeetechSMSServo.drive_speed's own setter/getter, in feetech.py).
    Forgetting the *50 made the published ramp 50x slower than the driver
    actually asked for -- masked offline because nothing here models the
    joint's own achievable speed, only a real simulator run could show the
    published target moving at the wrong rate.

    Constructed via __new__ rather than the real constructor, so this needs
    no rclpy node: set_drive_parameter only touches ``_condition`` and
    ``_drive_speed_mps``.
    """
    bus = IsaacJointBus.__new__(IsaacJointBus)
    bus._condition = threading.Condition()
    bus._drive_speed_mps = 0.0

    bus.set_drive_parameter(_DRIVE_SPEED_ADDR, 2450)  # wire word for 122500 steps/s

    assert bus._drive_speed_mps == pytest.approx(122500 / IsaacServo.TICKS_PER_METRE)
