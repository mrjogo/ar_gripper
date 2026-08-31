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

It also covers ``IsaacJointBus`` itself where that needs no simulator: where
the ramp's speed ceiling comes from, that the ramp respects it, that the first
measured sample seeds the goal, and that the subscription and the ramp timer
are wired into separate callback groups.

No rclpy node, no Isaac: the servo tests are driven by a fake
``IsaacJointBus`` whose ``wait_for_sample`` pops scripted ``(position_m,
velocity_mps, effort_N)`` tuples and whose ``command`` /
``set_drive_parameter`` just record, and the bus tests run the real
``IsaacJointBus`` constructor against a stub node.
"""

import threading
from pathlib import Path
from xml.etree import ElementTree

import pytest
from ar_gripper.scripts.isaac_servo import (
    _DRIVE_SPEED_ADDR,
    MOVING_DEADBAND_MPS,
    MOVING_WINDOW_S,
    SAMPLE_RESOLUTION_M,
    STALL_DWELL_S,
    STALL_POSITION_ERROR_M,
    WAIT_FOR_STOP_WINDOW_S,
    IsaacJointBus,
    IsaacServo,
    StallDetector,
    finger_velocity_limit_mps,
)
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState

SID = 1
JOINT_NAME = "primary_ar_gripper_body_finger1"
# The description in the source tree, not the installed copy: this test is
# about what the repository ships.
_PACKAGE_DIR = Path(__file__).resolve().parents[1] / "ar_gripper"
DESCRIPTION_PATH = (
    Path(__file__).resolve().parents[1] / "urdf" / "ar_gripper_macro.xacro"
)


class _ScriptedIsaacJointBus:
    """Records ``command()`` / ``set_drive_parameter()`` calls.

    ``wait_for_sample()`` pops scripted ``(position_m, velocity_mps,
    effort_N)`` tuples in order, repeating the last one once exhausted -- so a
    test that only cares about one steady state can script a single tuple and
    read the servo as many times as it likes.
    """

    def __init__(self, samples, stalled=False, moving=False):
        self._samples = list(samples)
        self.stalled = stalled
        # moving_sign comes off the bus rather than off the sample's velocity;
        # see StallDetector.update for why the simulator's velocity cannot
        # answer it.
        self.moving = moving
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

    def _make(samples, stalled=False):
        device = USB2FeetechDevice("/dev/fake")
        fake = fake_serials[-1]
        bus = _ScriptedIsaacJointBus(samples, stalled=stalled)
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


def _detector():
    """A ``StallDetector`` on the shipped thresholds."""
    return StallDetector(
        velocity_threshold_mps=MOVING_DEADBAND_MPS,
        position_error_m=STALL_POSITION_ERROR_M,
        dwell_s=STALL_DWELL_S,
    )


class _HardStopIsaacBus:
    """Ramps a target toward the last commanded goal, chases it with a
    first-order-ish finger, clamps at a hard stop and reports **no force at
    all** while pinned against it.

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

    The zero effort against the stop is the whole point of the fixture and not
    a simplification: that is what the simulator reports there, and it is why
    homing cannot be built on force. So this stub carries the shipped
    ``StallDetector`` rather than a stall force of its own -- the rule under
    test is the real one, and only the plant around it is fake.

    Time comes from the driver's own module clock, which is the same clock the
    real bus reads (a sim-time ``RosClock`` in the field, the deterministic
    ``FakeTime`` here). A dwell has to be measured against the timeline the
    driver paces its reads on, or a test can pass with a dwell no real homing
    run would ever satisfy.
    """

    STEP_M = 0.001  # metres the finger advances per wait_for_sample() call
    # The target is paid out faster than the finger travels, so the finger
    # runs a lag behind it exactly as the real one does -- which is what puts
    # a standing error on the far side of a hard stop for the detector to see.
    RAMP_STEP_M = 0.0016

    def __init__(self, closed_stop_m, start_offset_m=0.03):
        self._closed_stop_m = closed_stop_m
        self._position_m = closed_stop_m + start_offset_m
        self._target_m = self._position_m
        self._goal_m = self._position_m
        self.wait_for_sample_calls = 0
        self._stall = _detector()

    def wait_for_sample(self, timeout_s=None):
        from ar_gripper import gripper as gripper_module

        self.wait_for_sample_calls += 1
        delta = self._goal_m - self._target_m
        if abs(delta) <= self.RAMP_STEP_M:
            self._target_m = self._goal_m  # snap exactly, not via += (float noise)
        else:
            self._target_m += self.RAMP_STEP_M if delta > 0 else -self.RAMP_STEP_M
        reachable = max(self._target_m, self._closed_stop_m)
        delta = reachable - self._position_m
        if abs(delta) <= self.STEP_M:
            moved = delta
            self._position_m = reachable
        else:
            moved = self.STEP_M if delta > 0 else -self.STEP_M
            self._position_m += moved
        velocity = moved * 100.0  # nonzero while stepping, exactly 0.0 when settled
        self._stall.update(
            self._target_m, self._position_m, velocity, gripper_module.time.time()
        )
        return self._position_m, velocity, 0.0

    @property
    def stalled(self):
        return self._stall.stalled

    @property
    def moving(self):
        # The real bus answers moving_sign from the same detector, for the
        # reason given in StallDetector.update: the simulator's velocity cannot
        # be asked whether a joint has stopped.
        return self._stall.moving

    def command(self, position_m):
        self._goal_m = position_m

    def set_drive_parameter(self, addr, value):
        pass


# Where the finger is when homing starts, as an offset from the closed stop.
# Homing has to converge from all of them, and used not to: each attempt
# commands exactly 2047 ticks (25.9 mm) of closing from wherever the finger
# already is, so the overshoot past the stop -- the only thing an
# error-threshold rule can see -- is a function of the starting position, and
# there are starting positions that leave it arbitrarily small.
#
# 0.0 is where homing itself leaves the finger on the *previous* run (at the
# open end, which is this offset from the stop) once the stroke is homed, so a
# driver restart is exactly this case; 0.0259 is where the first attempt lands
# on the stop with essentially no overshoot at all.
_HOMING_START_OFFSETS_M = [0.0, 0.0018, 0.0259, 0.03, 0.05]


@pytest.mark.parametrize("start_offset_m", _HOMING_START_OFFSETS_M)
def test_isaac_backed_calibration_converges(fake_serials, fast_clock, start_offset_m):
    """Homing runs to completion against a stop that reports no force, from
    anywhere in the stroke.

    Same shape as test_mock_motor_current.py's
    test_calibrate_homes_against_an_obstruction, driven from Isaac-shaped
    samples rather than the internal obstruction model -- and with the load
    channel that homing keys on fed by motion rather than by force, because
    against a joint limit the simulator has no force to give it.

    ``Gripper._calibrate`` is unmodified and unmodifiable here: it is the real
    robot's homing sequence, shared with the hardware, and the substitution is
    entirely below the driver, in what the simulated servo reports.
    """
    from ar_gripper.feetech import USB2FeetechDevice
    from ar_gripper.gripper import Gripper

    device = USB2FeetechDevice("/dev/fake")
    fake = fake_serials[-1]
    fake.servos[SID] = IsaacServo(
        _HardStopIsaacBus(closed_stop_m=0.02, start_offset_m=start_offset_m)
    )
    gripper = Gripper(device, "primary", SID)

    assert gripper.calibrate() is True
    assert gripper.calibrated is True


def test_a_finger_that_reaches_its_target_is_never_called_stalled():
    """The failure mode the thresholds are jointly sized against.

    A finger settling into a reachable target spends most of its travel time
    creeping the last little way in -- the settle tail is around 70% of a
    stroke -- so "slow" alone is not evidence of anything, and a rule that
    treated it as evidence would report contact on every ordinary move. This
    walks the drive's own first-order response all the way to rest against a
    stationary target, sampling it at the publish rate, and asserts the
    detector stays quiet through every one of those samples.
    """
    detector = _detector()
    dt = 1.0 / IsaacJointBus.COMMAND_RATE_HZ
    tau = 0.62  # the drive's own time constant, as the stage reports it
    error = 0.019  # the lag a full-speed ramp leaves behind it

    for step in range(int(30.0 / dt)):
        velocity = error / tau
        assert not detector.update(0.0, -error, velocity, step * dt), (
            f"a freely settling finger was called stalled at t={step * dt:.2f} s, "
            f"error {error * 1e3:.3f} mm, velocity {velocity * 1e3:.3f} mm/s"
        )
        error -= velocity * dt
    assert error < 1e-9  # the trace really did settle, rather than stopping short


def _quantise(position_m):
    """Round a position the way Isaac's publisher does before it reaches us."""
    return round(position_m / SAMPLE_RESOLUTION_M) * SAMPLE_RESOLUTION_M


def test_a_settling_finger_is_still_moving_once_its_samples_start_repeating():
    """The regression: quantised samples repeat before the finger arrives.

    Isaac publishes joint positions on a 1e-4 m grid, so a finger closing on
    its target crosses a quantum boundary less often than once per sample well
    before it gets there -- and a rule that asked "did the reading change since
    the previous sample" answered "stopped" on a finger still 1.7 mm out. That
    is what Gripper._goto_position returns on, so the move ended early, the
    driver reported the position it really was at, and GripperCommand came back
    with reached_goal false against its own 1 mm tolerance.

    This walks the drive's first-order settle THROUGH the quantiser and asserts
    the detector keeps calling it moving until it is inside a fifth of that
    tolerance. Feeding un-quantised positions here would pass against either
    rule, which is exactly why the bug survived the suite it already had.
    """
    detector = _detector()
    dt = 1.0 / IsaacJointBus.COMMAND_RATE_HZ
    tau = 0.21  # the finger drive's time constant, as the stage reports it
    target = 0.05
    error = 0.0065  # the lag the ramp leaves behind at the end of a full stroke

    started_moving = False
    called_stopped_at = None
    for step in range(int(5.0 / dt)):
        position = _quantise(target - error)
        detector.update(target, position, error / tau, step * dt)
        # Only after it has been seen moving: `moving` starts False, and the
        # first sample of any trace has nothing to compare against.
        started_moving = started_moving or detector.moving
        if started_moving and not detector.moving and called_stopped_at is None:
            called_stopped_at = error
        error -= (error / tau) * dt

    assert started_moving, "the finger was never called moving in the first place"
    assert called_stopped_at is not None, "the finger was never called stopped"
    assert called_stopped_at < 2.0e-4, (
        f"the finger was called stopped {called_stopped_at * 1e3:.2f} mm from "
        "its target; Gripper.goto_position returns there and the driver's own "
        "reached_goal test allows 1 mm"
    )


def test_a_repeated_sample_alone_does_not_mean_stopped():
    """One repeat is the quantiser, not an arrival.

    The narrowest statement of the bug: two identical readings in a row, a
    third that has moved on. Under the old rule the middle sample reported
    stopped -- and one sample is all Gripper._goto_position needs.
    """
    detector = _detector()
    dt = 1.0 / IsaacJointBus.COMMAND_RATE_HZ

    detector.update(0.05, 0.0400, 0.01, 0.0)
    detector.update(0.05, 0.0401, 0.01, dt)
    detector.update(0.05, 0.0401, 0.01, 2 * dt)
    assert detector.moving, "a single repeated reading was taken for an arrival"
    detector.update(0.05, 0.0402, 0.01, 3 * dt)
    assert detector.moving


def test_a_joint_whose_reading_stays_put_is_stopped_after_the_window():
    """And the other side of it: the window does elapse.

    A finger that has genuinely arrived has to be reported as stopped, or no
    move ever completes. MOVING_WINDOW_S is how long that takes.
    """
    detector = _detector()
    dt = 1.0 / IsaacJointBus.COMMAND_RATE_HZ

    detector.update(0.05, 0.0499, 0.01, 0.0)
    detector.update(0.05, 0.0500, 0.01, dt)
    assert detector.moving

    now = dt
    while now < dt + MOVING_WINDOW_S:
        now += dt
        detector.update(0.05, 0.0500, 0.0, now)
    assert not detector.moving, (
        "a joint whose reading has not changed for a whole window is stopped"
    )


def test_the_first_quantum_crossing_refutes_stopped_at_the_start_of_a_move():
    """The start-of-move trap, with the window in place.

    A move begins with the joint not yet having moved, so it reads stopped --
    and the outrun limb is armed from the first sample, so the dwell starts
    counting immediately. What has to be true is that the joint refutes it long
    before the dwell elapses. At the shipped ramp speed the first quantum
    crossing lands ~3.2 ms in, against a dwell of 0.18 s.
    """
    detector = _detector()
    dt = 1.0 / IsaacJointBus.COMMAND_RATE_HZ
    speed = 0.031  # the description's finger velocity limit

    position = 0.0
    target = 0.0
    for step in range(int(STALL_DWELL_S / dt)):
        now = step * dt
        target = speed * now
        position = _quantise(speed * max(0.0, now - dt))
        assert not detector.update(target, position, speed, now), (
            f"a move that had just started was called stalled at t={now:.4f} s"
        )
    assert detector.moving, "the joint never registered as moving"


def test_a_finger_held_off_its_target_is_called_stalled_after_the_dwell():
    """The pressing arm: a target that has finished being paid out, sitting
    beyond a joint that is not moving.
    """
    detector = _detector()
    error = 2 * STALL_POSITION_ERROR_M

    assert not detector.update(error, 0.0, 0.0, 0.0)
    assert not detector.update(error, 0.0, 0.0, STALL_DWELL_S / 2)
    assert detector.update(error, 0.0, 0.0, STALL_DWELL_S)
    # And releases the moment the joint gets going again, so a move that was
    # blocked and then freed does not keep reporting contact. "Getting going" is
    # the position changing: at MOVING_DEADBAND_MPS one 120 Hz sample carries the
    # joint 0.8 um, eight times the epsilon. Velocity is deliberately left at the
    # value a pinned joint reports, because that is exactly the case where it
    # lies -- if this released on velocity alone it would release on a finger
    # that never moved.
    freed_m = MOVING_DEADBAND_MPS / 120.0
    assert not detector.update(error, freed_m, 0.0, STALL_DWELL_S + 0.01)


def test_a_joint_already_stopped_when_the_move_starts_needs_no_run_up():
    """The outrun arm, and the reason it exists.

    Every homing attempt after the first begins with the finger already
    against the stop and no error accumulated yet, and the driver only leaves
    WAIT_FOR_STOP_WINDOW_S between the move starting and reading the load. A
    rule that waited for the target to be paid out past an error threshold
    would spend a slice of that window that varies with how far the *previous*
    attempt happened to overshoot -- which is how homing came to work from some
    starting positions and fail from others.

    The ramp here is deliberately slow (2 mm/s), because that is what makes the
    difference visible: waiting for the error would take half a second before
    the dwell even started.
    """
    detector = _detector()
    dt = 1.0 / IsaacJointBus.COMMAND_RATE_HZ
    rate = 0.002
    target = 0.0
    latched_at = None

    for step in range(int(2.0 / dt)):
        now = step * dt
        if detector.update(target, 0.0, 0.0, now):
            latched_at = now
            break
        target -= rate * dt  # paid out away from a joint that never moves

    assert latched_at is not None, "a joint held off a moving target never stalled"
    assert latched_at <= STALL_DWELL_S + 2 * dt, (
        f"stalled only after {latched_at:.3f} s, which is the dwell plus a run-up; "
        f"the dwell alone is {STALL_DWELL_S:.3f} s and the window the driver "
        f"leaves is {WAIT_FOR_STOP_WINDOW_S:.3f} s"
    )


def test_the_dwell_fits_the_window_the_driver_leaves_for_it():
    """The budget, asserted rather than reasoned about in a comment.

    Gripper._wait_for_stop returns this long after the finger stops and homing
    reads the load immediately; a dwell that does not fit inside it is a stall
    homing never sees. Both sides are read off the driver, so this also fails
    if _wait_for_stop's own pacing changes underneath.
    """
    from ar_gripper.gripper import Gripper

    assert WAIT_FOR_STOP_WINDOW_S == pytest.approx(3 * Gripper._WAIT_CHECK_TIME_S)
    assert STALL_DWELL_S < WAIT_FOR_STOP_WINDOW_S


def test_the_stall_reaches_the_driver_as_load_and_as_current(make_isaac_servo):
    """Both of the driver's load-shaped questions, from the one hook.

    Homing reads ``present_load`` and the grasp loop reads ``present_current``.
    Feeding the stall through ``_contact_load`` is what keeps those two
    agreeing; overriding ``_load_now`` alone would leave the current model
    reporting an unloaded servo during a stall.
    """
    gripper, _bus, _isaac = make_isaac_servo([(0.0, 0.0, 0.0)], stalled=True)

    assert gripper.servo.present_load >= 10  # what _calibrate calls contact
    assert gripper.servo.present_current > IsaacServo.NO_LOAD_CURRENT_MA

    free, _bus, _isaac = make_isaac_servo([(0.0, 0.0, 0.0)], stalled=False)
    assert free.servo.present_load == 0
    assert free.servo.present_current == pytest.approx(
        IsaacServo.NO_LOAD_CURRENT_MA, abs=IsaacServo.CURRENT_LSB_MA
    )


def test_a_squeezed_object_still_reports_its_force(make_isaac_servo):
    """The stall must not displace the effort channel, only stand in where it
    has nothing to say. An object between the fingers IS an external load, so
    it reports as force, and a stall reading on top of it must not shrink a
    contact force that is larger.
    """
    hard = IsaacServo.MAX_EFFORT_N / 2
    gripper, _bus, _isaac = make_isaac_servo([(0.0, 0.0, hard)], stalled=True)

    assert gripper.servo.present_load == pytest.approx(50.0)


# What the finger actually reads, measured on the shipped asset with the
# grip-force tool that ships alongside the simulator stage, under Isaac 6.0.1.
# These are the
# two bounds LOAD_DEADBAND_N sits between, so they are written down here rather
# than left in a commit message: a change to the drive, the collider or the
# finger mass moves them, and this test is what says so.
FINGER_WEIGHT_BIAS_N = 0.3924  # empty jaws; == 0.040 kg * 9.81
GRASP_FREE_BODY_N = 1.599  # closing on a free 400 g, 40 mm cube
GRASP_IMMOVABLE_N = 3.547  # closing on an immovable 40 mm cube


def test_the_load_deadband_clears_the_finger_weight_but_not_a_grasp():
    """The threshold has a job with two sides, and it used to fail one of them.

    Below it sits the steady bias the finger's own weight puts on the measured
    effort, whose sign moves with the arm pose -- anything under that reports
    load on an empty gripper. Above it sits the weakest grasp that has to
    register. At 2.0 N the threshold sat BETWEEN the bias and a real grasp, so
    closing on a 400 g body reported exactly zero load.
    """
    assert IsaacServo.LOAD_DEADBAND_N > FINGER_WEIGHT_BIAS_N, (
        "an empty gripper would report load: the finger's own weight alone "
        f"reads {FINGER_WEIGHT_BIAS_N} N"
    )
    assert IsaacServo.LOAD_DEADBAND_N < GRASP_FREE_BODY_N, (
        "a real grasp would report zero: the weakest measured one develops "
        f"{GRASP_FREE_BODY_N} N"
    )


def test_a_grasp_of_a_free_body_reports_load_rather_than_zero(make_isaac_servo):
    """The regression, at the level the driver sees it.

    present_load is what homing reads and what a GripperCommand result carries
    as `effort`. A 400 g body held in the jaws has to move it off zero.
    """
    gripper, _bus, _isaac = make_isaac_servo([(0.018, 0.0, GRASP_FREE_BODY_N)])
    load = gripper.servo.present_load
    assert load > 0.0, "a grasp of a free 400 g body still reports no load at all"
    # Quantised on the way out through the servo's load register, so this checks
    # the magnitude rather than the exact quotient: a fraction of one percent of
    # stall torque, which is why homing still cannot run on it.
    assert load < 1.0
    assert load == pytest.approx(
        GRASP_FREE_BODY_N / IsaacServo.MAX_EFFORT_N * 100.0, abs=0.2
    )


def test_an_empty_gripper_still_reports_exactly_zero_load(make_isaac_servo):
    """And the other side: the finger's own weight is not a grasp.

    Reported as exactly 0.0 rather than something small, because that is the
    contract _contact_load documents and what the offline mock reports.
    """
    gripper, _bus, _isaac = make_isaac_servo([(0.05, 0.0, FINGER_WEIGHT_BIAS_N)])
    assert gripper.servo.present_load == 0.0


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


class _StubPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _StubTime:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1e9)

    def to_msg(self):
        return Time()


class _StubNode:
    """The slice of ``rclpy.node.Node`` ``IsaacJointBus.__init__`` actually uses.

    Enough to run the real constructor -- so these tests exercise the shipped
    wiring rather than a hand-assembled object that can drift away from it --
    without an rclpy context, an executor or a running simulator.
    """

    def __init__(self):
        self.published = _StubPublisher()
        self.subscription_callback = None
        self.subscription_group = None
        self.timer_callback = None
        self.timer_group = None
        self.timer_period_s = None
        self.warnings = []
        self.clock_s = 0.0

    def create_publisher(self, _msg_type, _topic, _qos):
        return self.published

    def create_subscription(
        self, _msg_type, _topic, callback, _qos, callback_group=None
    ):
        self.subscription_callback = callback
        self.subscription_group = callback_group
        return object()

    def create_timer(self, period_s, callback, callback_group=None):
        self.timer_period_s = period_s
        self.timer_callback = callback
        self.timer_group = callback_group
        return object()

    def get_clock(self):
        return self

    def now(self):
        """Advance one ramp tick per call, so the ramp integrates a known step."""
        self.clock_s += 1.0 / IsaacJointBus.COMMAND_RATE_HZ
        return _StubTime(self.clock_s)

    def get_logger(self):
        return self

    def warning(self, message, **_kwargs):
        self.warnings.append(message)


def make_bus(max_velocity_mps=0.031):
    node = _StubNode()
    bus = IsaacJointBus(
        node,
        joint_states_topic="/isaac/joint_states",
        command_topic="/isaac/gripper/joint_commands",
        joint_name=JOINT_NAME,
        max_velocity_mps=max_velocity_mps,
    )
    return bus, node


def sample(position_m, velocity_mps=0.0, effort_n=0.0):
    msg = JointState()
    msg.name = [JOINT_NAME]
    msg.position = [position_m]
    msg.velocity = [velocity_mps]
    msg.effort = [effort_n]
    return msg


def test_the_velocity_limit_comes_from_the_shipped_description():
    """The ramp ceiling is the joint's own limit, read out of the description
    instead of restated in Python. Fails if the description stops carrying a
    literal there (a xacro expression, or the attribute going away), which is
    the way this could silently start being re-derived from something else.
    """
    limit = finger_velocity_limit_mps(DESCRIPTION_PATH)

    assert limit > 0.0
    # Both fingers move together through one mechanism; a mimic joint carrying
    # a different ceiling than the joint it mimics is a description bug.
    velocities = {
        joint.find("limit").get("velocity")
        for joint in ElementTree.parse(DESCRIPTION_PATH).getroot().iter("joint")
        if "ar_gripper_body_finger" in (joint.get("name") or "")
    }
    assert velocities == {str(limit)}


def test_a_non_literal_velocity_limit_is_an_error_not_a_guess(tmp_path):
    description = tmp_path / "macro.xacro"
    description.write_text(
        '<robot><joint name="p_ar_gripper_body_finger1" type="prismatic">'
        '<limit effort="1000.0" lower="0.0" upper="0.05" '
        'velocity="${FINGER_VELOCITY}"/></joint></robot>'
    )

    with pytest.raises(ValueError, match="non-literal velocity limit"):
        finger_velocity_limit_mps(description)


def test_the_ramp_never_outruns_the_description_velocity_limit():
    """Publishing faster than the joint's limit just puts the target ahead of a
    joint that cannot follow it. The driver asks for far more than the limit
    (122500 steps/s is ~1.55 m/s against a limit of a few cm/s), so the clamp
    is load-bearing on every real move, and deleting it leaves no test red
    unless one asserts the published step directly.
    """
    limit = 0.02
    bus, node = make_bus(max_velocity_mps=limit)
    node.subscription_callback(sample(0.0))
    bus.set_drive_parameter(_DRIVE_SPEED_ADDR, 2450)  # 122500 steps/s, ~1.55 m/s
    bus.command(0.05)

    node.timer_callback()
    node.timer_callback()
    node.timer_callback()

    # The stub clock advances one tick period per call, and the ramp steps by
    # rate x measured interval -- so the first tick, which has no previous one
    # to measure against, moves nothing.
    step = limit / IsaacJointBus.COMMAND_RATE_HZ
    assert [msg.position[0] for msg in node.published.messages] == [
        pytest.approx(0.0),
        pytest.approx(step),
        pytest.approx(2 * step),
    ]


def test_the_first_sample_seeds_the_goal_instead_of_commanding_a_full_close():
    """0.0 m is the fully closed stop, not a neutral default. A bus that starts
    out believing the goal is 0.0 slams the finger shut the moment the driver
    starts, wherever it happens to be -- observed driving a finger from 0.019 m
    to 0.000 m on a restart with no goal ever sent.
    """
    bus, node = make_bus()

    # Before any sample there is no defensible target, so nothing is published.
    node.timer_callback()
    assert node.published.messages == []

    node.subscription_callback(sample(0.019))
    bus.set_drive_parameter(_DRIVE_SPEED_ADDR, 2450)
    node.timer_callback()

    assert node.published.messages[-1].position[0] == pytest.approx(0.019)


def test_the_bus_reports_a_stall_against_the_target_it_publishes():
    """Not just that the rule works -- that the bus feeds it the right target.

    The error has to be measured against the target the ramp has actually
    published, because the goal can be a whole stroke ahead of it while the
    ramp is still paying it out, and measuring against the goal would report a
    stall on every long move before the finger had been asked to go anywhere.

    Which is why the ramp is deliberately left un-ticked here for longer than
    the dwell, with the goal a whole stroke away: against the published target
    there is nothing to see (a quarter of a millimetre from the joint, and not
    moving), while against the goal this is 50 mm of error on a joint that is
    not moving. The stub node's clock advances one publish period per call.
    """
    bus, node = make_bus()
    node.subscription_callback(sample(0.0))
    bus.set_drive_parameter(_DRIVE_SPEED_ADDR, 2450)
    bus.command(-0.05)
    node.timer_callback()  # the first tick has no interval to integrate
    node.timer_callback()  # one publish period of ramp: ~0.26 mm

    samples = int(3 * STALL_DWELL_S * IsaacJointBus.COMMAND_RATE_HZ)
    for _ in range(samples):
        node.subscription_callback(sample(0.0))
    assert bus.stalled is False

    # Let the ramp run past the joint, holding the joint still, and the same
    # samples become a stall.
    for _ in range(samples):
        node.timer_callback()
        node.subscription_callback(sample(0.0))
    assert bus.stalled is True


def test_the_subscription_and_the_ramp_timer_get_their_own_callback_groups():
    """Both must stay out of the node's default group and out of each other's.

    A blocking read inside a callback in the node's default group (the status
    timer does exactly that) would otherwise hold off the subscription that
    delivers the sample it is blocked on, and the ramp timer and the
    subscription fire at comparable rates, so serialising them against each
    other recreates a smaller version of the same contention.
    """
    _bus, node = make_bus()

    assert node.subscription_group is not None
    assert node.timer_group is not None
    assert node.subscription_group is not node.timer_group


def test_the_bus_takes_its_velocity_ceiling_from_the_description_reader(monkeypatch):
    """Not just that the reader works and the clamp works -- that they are WIRED.

    Both halves passing while the constructor quietly holds a literal is the
    exact failure the derivation exists to prevent, and it is invisible for as
    long as the literal happens to match the description.
    """
    from ar_gripper.scripts import isaac_servo

    sentinel = 0.12345
    monkeypatch.setattr(
        isaac_servo, "finger_velocity_limit_mps", lambda *_args, **_kwargs: sentinel
    )

    bus = IsaacJointBus(
        _StubNode(),
        joint_states_topic="/isaac/joint_states",
        command_topic="/isaac/gripper/joint_commands",
        joint_name=JOINT_NAME,
    )

    assert bus.max_velocity_mps == sentinel


def test_the_description_reader_loads_by_path_with_no_ros_available():
    """The simulator stage loads this reader out of the checkout, by file path.

    It runs inside Isaac's own interpreter, which must not have a ROS overlay
    sourced, and it does that specifically so its staleness check and the
    driver read the description through the same code instead of through two
    parsers that can drift apart. That only works for as long as
    ``ar_gripper/description.py`` stays importable with nothing but the
    standard library -- so this loads it exactly the way the stage does, with
    every ROS module poisoned and the package not on the path at all.
    """
    import subprocess
    import sys

    reader = _PACKAGE_DIR / "description.py"
    code = (
        "import importlib.util, sys\n"
        "for m in ['rclpy', 'ament_index_python', 'sensor_msgs', 'rcl_interfaces']:\n"
        "    sys.modules[m] = None\n"
        f"spec = importlib.util.spec_from_file_location('d', r'{reader}')\n"
        "module = importlib.util.module_from_spec(spec)\n"
        "spec.loader.exec_module(module)\n"
        f"print(module.finger_velocity_limit_mps(r'{DESCRIPTION_PATH}'))\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
        env={"PATH": ""},
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert float(result.stdout.strip()) == finger_velocity_limit_mps(DESCRIPTION_PATH)
