"""The one derivation of "did it reach the goal, or stop on something?".

Both the GripperCommand action result and the ~/gripper_state topic report this,
and they must not be able to disagree about the same instant, so both go through
``ARGripperStandalone.derive_grasp_state``. These tests pin it directly, in
plain numbers, with no bus and no ROS involved.

The direction cases are the reason the status is not a bool: stopped short while
OPENING is a jam, stopped short while CLOSING is a grasp.
"""

import pytest

from ar_gripper.standalone import ARGripperStandalone as SA

CLOSED = SA.FINGER_CLOSED_POS  # 0.0 m
OPEN = SA.FINGER_OPEN_POS  # 0.05 m
GRASP_FORCE_N = 300.0


def test_reached_goal_reports_no_object():
    state = SA.derive_grasp_state(
        position_m=0.02, effort_N=0.0, goal_position_m=0.02, calibrated=True
    )
    assert state.reached_goal is True
    assert state.stalled is False
    assert state.object_detection_status == SA.OBJECT_DETECTION_NO_OBJECT


def test_reached_goal_while_still_squeezing_is_still_no_object():
    """Holding torque against an object AT the commanded position is not a stall."""
    state = SA.derive_grasp_state(
        position_m=0.02, effort_N=GRASP_FORCE_N, goal_position_m=0.02, calibrated=True
    )
    assert state.reached_goal is True
    assert state.stalled is False
    assert state.object_detection_status == SA.OBJECT_DETECTION_NO_OBJECT


def test_goal_tolerance_is_a_millimetre():
    assert SA.derive_grasp_state(0.0209, 0.0, 0.02, True).reached_goal is True
    assert SA.derive_grasp_state(0.0212, 0.0, 0.02, True).reached_goal is False


def test_blocked_while_closing_detects_an_object():
    """Commanded shut, stopped part-open under load: something is in the fingers."""
    state = SA.derive_grasp_state(
        position_m=0.014,
        effort_N=GRASP_FORCE_N,
        goal_position_m=CLOSED,
        calibrated=True,
    )
    assert state.reached_goal is False
    assert state.stalled is True
    assert state.object_detection_status == SA.OBJECT_DETECTION_DETECTED_CLOSING


def test_blocked_while_opening_detects_an_object():
    """Same stall, opposite direction -- jammed, not grasped."""
    state = SA.derive_grasp_state(
        position_m=0.014, effort_N=GRASP_FORCE_N, goal_position_m=OPEN, calibrated=True
    )
    assert state.reached_goal is False
    assert state.stalled is True
    assert state.object_detection_status == SA.OBJECT_DETECTION_DETECTED_OPENING


def test_no_goal_yet_is_unknown():
    """Nothing has been commanded, so "before the requested position" has no meaning."""
    state = SA.derive_grasp_state(
        position_m=0.014, effort_N=GRASP_FORCE_N, goal_position_m=None, calibrated=True
    )
    assert state.reached_goal is False
    assert state.stalled is False
    assert state.object_detection_status == SA.OBJECT_DETECTION_UNKNOWN


def test_uncalibrated_is_unknown():
    """Position is not referenced to anything, so neither is "reached the goal"."""
    state = SA.derive_grasp_state(
        position_m=0.02, effort_N=0.0, goal_position_m=0.02, calibrated=False
    )
    assert state.object_detection_status == SA.OBJECT_DETECTION_UNKNOWN


def test_short_of_the_goal_under_no_load_is_unknown():
    """Still travelling: nothing is decided until the fingers settle."""
    state = SA.derive_grasp_state(
        position_m=0.03, effort_N=0.0, goal_position_m=CLOSED, calibrated=True
    )
    assert state.reached_goal is False
    assert state.stalled is False
    assert state.object_detection_status == SA.OBJECT_DETECTION_UNKNOWN


def test_status_constants_match_the_message():
    """standalone.py is ROS-free and cannot import the msg, so the values are copied.

    They are only useful if they stay equal to the ones the message publishes.
    """
    msg = pytest.importorskip("ar_gripper_interfaces.msg")
    GripperState = msg.GripperState
    assert SA.OBJECT_DETECTION_UNKNOWN == GripperState.UNKNOWN
    assert SA.OBJECT_DETECTION_DETECTED_OPENING == GripperState.DETECTED_OPENING
    assert SA.OBJECT_DETECTION_DETECTED_CLOSING == GripperState.DETECTED_CLOSING
    assert SA.OBJECT_DETECTION_NO_OBJECT == GripperState.NO_OBJECT


# --------------------------------------------------------------------------- #
# The goal the derivation is measured against, tracked on the standalone
# --------------------------------------------------------------------------- #
def test_goal_starts_unset(make_standalone):
    sa, _fake, _path = make_standalone()
    assert sa.goal_position_m is None
    assert sa.grasp_state().object_detection_status == SA.OBJECT_DETECTION_UNKNOWN


def test_close_records_the_goal(make_standalone):
    sa, _fake, _path = make_standalone()
    sa.close()
    assert sa.goal_position_m == CLOSED


def test_open_records_the_goal(make_standalone):
    sa, _fake, _path = make_standalone()
    sa.open()
    assert sa.goal_position_m == OPEN


def test_set_goal_records_the_goal_in_metres(make_standalone):
    sa, _fake, _path = make_standalone()
    sa.set_goal(50.0, unit="percent")
    assert sa.goal_position_m == pytest.approx(0.025)


def test_release_clears_the_goal(make_standalone):
    """Slack fingers are not on their way anywhere, so no goal is outstanding."""
    sa, _fake, _path = make_standalone()
    sa.close()
    sa.release()
    assert sa.goal_position_m is None
    assert sa.grasp_state().object_detection_status == SA.OBJECT_DETECTION_UNKNOWN


def test_calibrate_clears_the_goal(make_standalone):
    """Homing re-references position; whatever was commanded before is meaningless."""
    sa, _fake, _path = make_standalone(load_sequence=[5, 15, 15, 0])
    sa.close()
    sa.calibrate()
    assert sa.goal_position_m is None


def test_grasp_state_uses_live_reads_against_the_tracked_goal(make_standalone):
    """A free close reaches the goal -> NO_OBJECT, straight off the (fake) bus."""
    sa, _fake, _path = make_standalone()
    sa.close()
    state = sa.grasp_state()
    assert state.reached_goal is True
    assert state.object_detection_status == SA.OBJECT_DETECTION_NO_OBJECT


def test_grasp_state_accepts_an_already_taken_snapshot(make_standalone):
    """So a caller that needs both does not pay for two rounds of bus reads."""
    sa, _fake, _path = make_standalone()
    sa.close()
    snapshot = sa.get_state()
    assert sa.grasp_state(snapshot) == sa.derive_grasp_state(
        snapshot["position_m"],
        snapshot["effort_N"],
        sa.goal_position_m,
        snapshot["calibrated"],
    )
