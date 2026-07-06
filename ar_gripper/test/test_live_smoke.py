"""Level 6: optional live smoke test against real hardware.

Never runs by default. Enable with ``AR_GRIPPER_LIVE=1`` (read-only: connect and
read the present position). A small supervised open/close/release additionally
requires ``AR_GRIPPER_LIVE_MOVE=1`` and physically moves the gripper — only run
it with the hardware clear and supervised.

    AR_GRIPPER_LIVE=1 AR_GRIPPER_PORT=/dev/ttyUSB0 pytest test/test_live_smoke.py
    AR_GRIPPER_LIVE=1 AR_GRIPPER_LIVE_MOVE=1 pytest test/test_live_smoke.py
"""

import os

import pytest

from ar_gripper.standalone import ARGripperStandalone

LIVE = os.environ.get("AR_GRIPPER_LIVE") == "1"
LIVE_MOVE = os.environ.get("AR_GRIPPER_LIVE_MOVE") == "1"
PORT = os.environ.get("AR_GRIPPER_PORT", "/dev/ttyUSB0")
SERVO_ID = int(os.environ.get("AR_GRIPPER_SERVO_ID", "1"))

pytestmark = pytest.mark.skipif(
    not LIVE, reason="live hardware test; set AR_GRIPPER_LIVE=1 to enable"
)


def test_live_read_only():
    """Connect without a physical rehome and read state (no motion)."""
    gripper = ARGripperStandalone(
        port=PORT, servo_id=SERVO_ID, name="primary", calibrate_on_init=False
    )
    ticks = gripper.get_position("ticks")
    state = gripper.get_state()
    print(f"live present position: {ticks} ticks, state: {state}")
    assert isinstance(ticks, int)
    assert state["ticks"] == gripper.get_position("ticks")


@pytest.mark.skipif(
    not LIVE_MOVE, reason="physical motion; set AR_GRIPPER_LIVE_MOVE=1 to enable"
)
def test_live_supervised_open_close():
    """Supervised open -> close -> release. Physically moves the gripper."""
    gripper = ARGripperStandalone(port=PORT, servo_id=SERVO_ID, name="primary")
    assert gripper.is_calibrated
    assert gripper.open() is True
    assert gripper.close() is True
    assert gripper.release() is True
