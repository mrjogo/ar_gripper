"""Finger service: driving the fingers out of the carriage, and new ones back in.

A finger that jams mid-close leaves two problems behind. The fingers are stuck
in the carriage, and the position calibration is wrong because the servo spent
a while pushing against something that was not the hard stop it thought it was.

Getting out of that is a two-handed job. Pulling fingers out is a move: the
driver keeps driving toward open, past the calibrated open stop, while a
person pulls, and it does not go through ``goto_position`` -- it is driven
against the person doing the work, and stops when it is told to or when its
own bound runs out. Putting new ones in is not a move at all: the two fingers
are racks on one pinion, so seating them needs the pinion free to turn while a
person pushes each one in by hand, so the driver's part of that is just
cutting torque.

These are ``Gripper`` motions, exercised here against the in-memory Feetech bus
with no ROS and no hardware. The node-level half (the parameter, the refusals,
the services) is in test_node_finger_service.py.
"""

import time as real_time

import pytest

SID = 1

# Register addresses, as ar_gripper.feetech writes them.
MIN_POSITION_LIMIT = 0x09
MAX_POSITION_LIMIT = 0x0B
POSITION_CORRECTION = 0x1F
TORQUE_SWITCH = 0x28
GOAL_POSITION = 0x2A
TORQUE_LIMIT = 0x30

RESET_MIDPOINT = 128  # written to TORQUE_SWITCH; re-references position to 2048
TORQUE_OFF = 0


def _writes(fake, servo_id=SID):
    """Every register write on the bus, in order, as ``(address, [bytes])``."""
    out = []
    for packet in fake.trace:
        if len(packet) < 6 or packet[0] != 0xFF or packet[1] != 0xFF:
            continue
        if packet[2] != servo_id:
            continue
        instruction = packet[4 : 4 + (packet[3] - 1)]
        if not instruction or instruction[0] != 0x03:  # 0x03 == write
            continue
        out.append((instruction[1], list(instruction[2:])))
    return out


def _word(data):
    return data[0] | data[1] << 8


def _goal(data):
    value = data[0] | (data[1] & 0x7F) << 8
    return -value if data[1] & 0x80 else value


def _goals(fake):
    return [_goal(data) for addr, data in _writes(fake) if addr == GOAL_POSITION]


def _resets(fake):
    return [
        data
        for addr, data in _writes(fake)
        if addr == TORQUE_SWITCH and data == [RESET_MIDPOINT]
    ]


def _wait_for(predicate, timeout_s=10.0):
    """Poll a predicate in REAL time (the driver clock under test is not real)."""
    deadline = real_time.monotonic() + timeout_s
    while real_time.monotonic() < deadline:
        if predicate():
            return True
        real_time.sleep(0.005)
    return False


@pytest.fixture
def service_gripper(make_gripper):
    """A calibrated gripper on a fake bus, with any open drive stopped on teardown."""
    gripper, fake = make_gripper(servo_id=SID, calibrated=True)
    yield gripper, fake
    gripper.finger_service_open_stop()


# --------------------------------------------------------------------------- #
# Constants
# --------------------------------------------------------------------------- #
def test_the_service_torque_is_the_calibration_torque():
    """The torque already chosen for pushing the carriage into a hard stop."""
    from ar_gripper.gripper import Gripper

    assert Gripper.FINGER_SERVICE_TORQUE == Gripper._CALIBRATION_TORQUE
    assert Gripper.FINGER_SERVICE_MAX_S == 60.0


# --------------------------------------------------------------------------- #
# Open: keep driving toward open, past the stop, until told to stop
# --------------------------------------------------------------------------- #
def _seed_calibrated_limits(gripper, fake):
    """Leave the servo exactly as ``_calibrate`` leaves it, and clear the trace.

    Not an arbitrary starting state: it is the one the gripper is actually in
    when an operator decides the fingers need to come out. From here only
    ``min_position_limit`` differs from what the service motion wants, which is
    the point -- it is also the only one of the three that would refuse the
    goal.
    """
    servo = gripper.servo
    servo.min_position_limit = gripper._POSITION_MIN
    servo.max_position_limit = gripper._POSITION_MAX
    servo.position_correction = 0
    fake.trace.clear()


def test_the_open_drive_lifts_the_calibrated_limits_before_it_moves(
    service_gripper, frozen_clock
):
    """The calibrated stroke is about to stop being true, and it blocks the goal."""
    gripper, fake = service_gripper
    _seed_calibrated_limits(gripper, fake)

    assert gripper.finger_service_open() is True
    assert _wait_for(lambda: _goals(fake))

    # Nothing of the calibrated stroke is left in force.
    assert gripper.servo.min_position_limit == 0
    assert gripper.servo.max_position_limit == 4095
    assert gripper.servo.position_correction == 0
    assert gripper.servo.torque_limit == pytest.approx(gripper.FINGER_SERVICE_TORQUE)

    # And the one register that would have refused the goal was lifted BEFORE
    # the goal was written, not after it.
    writes = _writes(fake)
    first_goal = next(i for i, (addr, _) in enumerate(writes) if addr == GOAL_POSITION)
    before_the_move = writes[:first_goal]

    assert (MIN_POSITION_LIMIT, [0, 0]) in before_the_move
    assert any(
        addr == TORQUE_LIMIT and _word(data) == int(gripper.FINGER_SERVICE_TORQUE * 10)
        for addr, data in before_the_move
    )
    # The other two were already where the service wants them, so they cost no
    # EEPROM write at all -- see test_the_open_drive_lifts_limits_that_are_wrong....
    already_right = [addr for addr, _ in writes]
    assert MAX_POSITION_LIMIT not in already_right
    assert POSITION_CORRECTION not in already_right


def test_the_open_drive_lifts_limits_that_are_wrong_rather_than_merely_stale(
    service_gripper, frozen_clock
):
    """`_set_if_different` skips a correct register; it must not skip a wrong one."""
    gripper, fake = service_gripper
    gripper.servo.min_position_limit = gripper._POSITION_MIN
    gripper.servo.max_position_limit = 1000
    gripper.servo.position_correction = 100

    gripper.finger_service_open()
    assert _wait_for(lambda: _goals(fake))

    assert gripper.servo.min_position_limit == 0
    assert gripper.servo.max_position_limit == 4095
    assert gripper.servo.position_correction == 0


def test_the_open_drive_goes_past_the_calibrated_open_stop(
    service_gripper, frozen_clock
):
    """Open is the low-count side, and the goal is below the calibrated open stop."""
    from ar_gripper.gripper import Gripper

    gripper, fake = service_gripper

    gripper.finger_service_open()
    assert _wait_for(lambda: _goals(fake))

    assert set(_goals(fake)) == {0}
    assert 0 < Gripper._POSITION_MIN  # i.e. the goal is past the calibrated stop


def test_the_open_drive_re_references_the_encoder_before_commanding(
    service_gripper, frozen_clock
):
    """ "Toward 0" only means something once the encoder has a known zero."""
    gripper, fake = service_gripper

    gripper.finger_service_open()
    assert _wait_for(lambda: _goals(fake))

    writes = _writes(fake)
    first_goal = next(i for i, (addr, _) in enumerate(writes) if addr == GOAL_POSITION)
    assert (TORQUE_SWITCH, [RESET_MIDPOINT]) in writes[:first_goal]


def test_the_open_drive_keeps_pushing_rather_than_arriving_once(
    service_gripper, frozen_clock
):
    """It re-arms when it runs out of travel, so the push does not end on arrival."""
    gripper, fake = service_gripper

    gripper.finger_service_open()
    assert _wait_for(lambda: len(_goals(fake)) >= 3)

    # Every re-arm is a fresh re-reference: the encoder gets a new 2048 ticks
    # of room toward open, so the drive is not bounded by the stroke.
    assert len(_resets(fake)) >= 3


def test_the_open_drive_stops_when_it_is_told_to_and_goes_slack(
    service_gripper, frozen_clock
):
    gripper, fake = service_gripper

    gripper.finger_service_open()
    assert _wait_for(lambda: _goals(fake))
    assert gripper.finger_service_open_active is True

    position = gripper.finger_service_open_stop()

    assert isinstance(position, int)
    assert gripper.finger_service_open_active is False
    assert _writes(fake)[-1] == (TORQUE_SWITCH, [TORQUE_OFF])


def test_the_open_drive_stops_on_its_own_when_its_bound_runs_out(service_gripper):
    """``fast_clock`` runs the driver's clock past FINGER_SERVICE_MAX_S at once."""
    gripper, fake = service_gripper

    gripper.finger_service_open()

    assert _wait_for(lambda: not gripper.finger_service_open_active)
    assert _writes(fake)[-1] == (TORQUE_SWITCH, [TORQUE_OFF])


def test_the_open_drive_refuses_to_start_twice(service_gripper, frozen_clock):
    gripper, _fake = service_gripper

    assert gripper.finger_service_open() is True
    assert gripper.finger_service_open() is False


def test_stopping_an_open_drive_that_never_started_still_goes_slack(service_gripper):
    """The operator's hands are in the machine; "stop" means slack either way."""
    gripper, fake = service_gripper

    gripper.finger_service_open_stop()

    assert _writes(fake)[-1] == (TORQUE_SWITCH, [TORQUE_OFF])


# --------------------------------------------------------------------------- #
# Close: free the pinion so new fingers can be pushed in by hand
# --------------------------------------------------------------------------- #
def test_the_close_only_cuts_torque(service_gripper):
    """No goal, no re-reference, no limit change -- there is nothing to drive."""
    gripper, fake = service_gripper
    fake.trace.clear()  # drop the constructor's own bring-up writes

    gripper.finger_service_close()

    assert _writes(fake) == [(TORQUE_SWITCH, [TORQUE_OFF])]


def test_the_close_returns_immediately(service_gripper, frozen_clock):
    """The clock under test never advances; a close that waited on it would hang."""
    gripper, _fake = service_gripper

    gripper.finger_service_close()  # hangs here if this still polls a deadline


def test_the_close_reports_the_servo_position(service_gripper):
    gripper, fake = service_gripper
    fake.servo(SID).present_position_value = 1234

    position = gripper.finger_service_close()

    assert position == 1234


def test_the_close_can_be_called_repeatedly(service_gripper):
    gripper, fake = service_gripper
    fake.trace.clear()  # drop the constructor's own bring-up writes

    gripper.finger_service_close()
    gripper.finger_service_close()

    assert _writes(fake) == [
        (TORQUE_SWITCH, [TORQUE_OFF]),
        (TORQUE_SWITCH, [TORQUE_OFF]),
    ]


# --------------------------------------------------------------------------- #
# When something fails mid-motion: torque comes off anyway
#
# The fingers are being held by a person. A driver that stops driving because
# it crashed is fine; a driver that crashes while still driving is not, and
# nothing above the driver can tell the difference from the outside.
# --------------------------------------------------------------------------- #
def _fail_bus_reads_of(servo, addr):
    """Make the fake servo's reads of one register fail, as a dying bus does.

    Reads only: ``release()`` is a WRITE, so this leaves the driver able to cut
    torque -- which is the whole question. A bus too broken to write at all is
    a different failure, and there is nothing the driver can do about that one.
    """
    import serial

    original = servo.read

    def read(address, count):
        if address == addr:
            raise serial.SerialException("injected bus failure")
        return original(address, count)

    servo.read = read


def test_a_bus_failure_mid_drive_still_cuts_torque(service_gripper, frozen_clock):
    """Otherwise the thread vanishes with torque enabled and goal 0 standing."""
    from ar_gripper.mock import FakeServo

    gripper, fake = service_gripper
    _fail_bus_reads_of(fake.servo(SID), FakeServo.PRESENT_POSITION)

    gripper.finger_service_open()

    assert _wait_for(lambda: not gripper.finger_service_open_active)
    assert _goals(fake) == [0]  # it did start driving, then lost the bus
    assert _writes(fake)[-1] == (TORQUE_SWITCH, [TORQUE_OFF])


# --------------------------------------------------------------------------- #
# What it costs: the calibration is gone
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("motion", ["open", "close"])
def test_a_service_motion_marks_the_gripper_uncalibrated(service_gripper, motion):
    """Whatever position meant before, it does not mean it afterwards."""
    gripper, _fake = service_gripper
    assert gripper.calibrated is True

    if motion == "open":
        gripper.finger_service_open()
    else:
        gripper.finger_service_close()

    assert gripper.calibrated is False
