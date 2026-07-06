"""Level 2 (characterization): the unchanged feetech/gripper stack emits exactly
the golden byte traces captured from this branch's baseline.

These tests pin the *current* wire behaviour so the standalone refactor can be
proven byte-identical against the same goldens (see test_standalone.py). They
exercise only ``ar_gripper.feetech`` / ``ar_gripper.gripper`` (no ROS) and pass
on the pre-refactor code.
"""

import json

import pytest
from conftest import FIXTURES_DIR

GOLDEN = json.loads((FIXTURES_DIR / "wire_traces_golden.json").read_text())
SID = GOLDEN["servo_id"]


def _hex(trace):
    return [packet.hex() for packet in trace]


def test_construction_handshake_and_init_servo_trace(make_gripper):
    """FeetechSMSServo handshake + Gripper._init_servo write the golden packets."""
    _gripper, fake = make_gripper(servo_id=SID)
    assert _hex(fake.trace) == GOLDEN["construct"]


@pytest.mark.parametrize("name", sorted(GOLDEN["reads"]))
def test_register_read_packets(make_gripper, name):
    """Each servo register read emits its exact golden request packet."""
    gripper, fake = make_gripper(servo_id=SID)
    fake.trace.clear()
    getattr(gripper.servo, name)
    assert _hex(fake.trace) == GOLDEN["reads"][name]


def test_release_trace(make_gripper):
    gripper, fake = make_gripper(servo_id=SID)
    fake.trace.clear()
    gripper.release()
    assert _hex(fake.trace) == GOLDEN["ops"]["release"]


def test_halt_trace(make_gripper):
    gripper, fake = make_gripper(servo_id=SID)
    fake.trace.clear()
    gripper.halt()
    assert _hex(fake.trace) == GOLDEN["ops"]["halt"]


def test_set_torque_trace(make_gripper):
    gripper, fake = make_gripper(servo_id=SID)
    fake.trace.clear()
    gripper.set_torque(50)
    assert _hex(fake.trace) == GOLDEN["ops"]["set_torque_50"]


def test_write_goal_position_trace(make_gripper):
    gripper, fake = make_gripper(servo_id=SID)
    fake.trace.clear()
    gripper.servo.goal_position = 2048
    assert _hex(fake.trace) == GOLDEN["ops"]["write_goal_2048"]


def test_open_trace(make_gripper):
    gripper, fake = make_gripper(servo_id=SID, calibrated=True)
    fake.trace.clear()
    gripper.open()
    assert _hex(fake.trace) == GOLDEN["ops"]["open"]


def test_goto_position_trace(make_gripper):
    gripper, fake = make_gripper(servo_id=SID, calibrated=True)
    fake.trace.clear()
    gripper.goto_position(50, 50)
    assert _hex(fake.trace) == GOLDEN["ops"]["goto_50_50"]


def test_calibrate_trace(make_gripper):
    gripper, fake = make_gripper(servo_id=SID, load_sequence=[5, 15, 15, 0])
    fake.trace.clear()
    assert gripper.calibrate() is True
    assert _hex(fake.trace) == GOLDEN["ops"]["calibrate"]
