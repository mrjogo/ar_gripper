"""The fake servo's motor-current model, and the stall branch it makes reachable.

``Gripper._goto_position`` applies holding torque when EITHER two consecutive current
samples exceed 1.4x the baseline OR the servo reports it has stopped, and both are
checked in the same loop iteration -- so "a stall was detected" cannot be read off the
return value. It is read off the wire instead: a stall fires while the servo is still
reporting movement, so ``moving_sign`` reads follow the holding-torque write.
"""

import pytest

from ar_gripper.mock import checksum

SID = 1
HOLDING_TORQUE_RAW = 100  # Gripper.HOLDING_TORQUE (10 %) at the register's 0.1 %/LSB
OBSTRUCTION = 3000  # ticks; between the open (150) and closed (4095) extremes


def _read_packet(addr, n):
    body = [SID, 4, 0x02, addr, n]
    return bytes([0xFF, 0xFF] + body + [checksum(body)])


def _write_word_packet(addr, value):
    body = [SID, 5, 0x03, addr, value & 0xFF, (value >> 8) & 0xFF]
    return bytes([0xFF, 0xFF] + body + [checksum(body)])


MOVING_SIGN_READ = _read_packet(0x42, 1)
HOLDING_TORQUE_WRITE = _write_word_packet(0x30, HOLDING_TORQUE_RAW)


@pytest.fixture
def make_servo(fake_serials, fast_clock):
    """Build a calibrated Gripper over a FakeServo configured for this test."""
    from ar_gripper.feetech import USB2FeetechDevice
    from ar_gripper.gripper import Gripper
    from ar_gripper.mock import FakeServo

    def _make(**servo_kwargs):
        device = USB2FeetechDevice("/dev/fake")
        fake = fake_serials[-1]
        fake.servos[SID] = FakeServo(**servo_kwargs)
        gripper = Gripper(device, "primary", SID)
        gripper._calibrated = True
        fake.trace.clear()
        return gripper, fake

    return _make


def _moving_reads_after_holding_torque(trace):
    if HOLDING_TORQUE_WRITE not in trace:
        return None
    start = trace.index(HOLDING_TORQUE_WRITE) + 1
    return sum(1 for packet in trace[start:] if packet == MOVING_SIGN_READ)


def test_no_load_current_is_not_degenerate(make_servo):
    """A zero baseline would make the 1.4x ratio test a hair trigger."""
    gripper, _ = make_servo(travel_ticks_per_read=100)
    assert gripper.servo.present_current == pytest.approx(65.0)
    assert gripper.servo.present_current > 0.0


def test_current_is_quantised_to_the_wire(make_servo):
    """Readings are multiples of 6.5 mA because feetech.py decodes raw * 6.5."""
    gripper, fake = make_servo(travel_ticks_per_read=100)
    fake.servos[SID].no_load_current_ma = 70.0  # not a multiple of 6.5
    assert gripper.servo.present_current == pytest.approx(71.5)  # round(70/6.5) * 6.5


def test_five_baseline_reads_see_five_different_samples(make_servo):
    """The regression guard against a cached getter collapsing the baseline window."""
    gripper, _ = make_servo(travel_ticks_per_read=100, obstruction_ticks=OBSTRUCTION)
    gripper.servo.goal_position = 4095
    positions = [gripper.servo.present_position for _ in range(5)]
    assert len(set(positions)) == 5


def test_load_is_exactly_zero_while_free(make_servo):
    """_calibrate's retreat loop is `while present_load > 0` with five tries."""
    gripper, _ = make_servo(travel_ticks_per_read=100, obstruction_ticks=OBSTRUCTION)
    assert gripper.servo.present_load == 0
    assert isinstance(gripper.servo.present_load, float)


def test_closing_onto_an_obstruction_stalls_at_it(make_servo):
    gripper, fake = make_servo(travel_ticks_per_read=100, obstruction_ticks=OBSTRUCTION)
    assert gripper.goto_position(0, 50) is True
    assert gripper.servo.present_position == OBSTRUCTION
    assert gripper.servo.present_load == pytest.approx(30.0)
    assert _moving_reads_after_holding_torque(fake.trace) == 4


def test_closing_freely_never_declares_a_stall(make_servo):
    gripper, fake = make_servo(travel_ticks_per_read=100)
    assert gripper.goto_position(0, 50) is True
    assert gripper.servo.present_position == 4095
    assert _moving_reads_after_holding_torque(fake.trace) == 0


def test_calibrate_homes_against_an_obstruction(fake_serials, fast_clock):
    """Homing with no scripted load_sequence at all -- contact comes from geometry."""
    from ar_gripper.feetech import USB2FeetechDevice
    from ar_gripper.gripper import Gripper
    from ar_gripper.mock import FakeServo

    device = USB2FeetechDevice("/dev/fake")
    fake = fake_serials[-1]
    fake.servos[SID] = FakeServo(travel_ticks_per_read=200, obstruction_ticks=3800)
    gripper = Gripper(device, "primary", SID)
    assert gripper.calibrate() is True
    assert gripper.calibrated is True
