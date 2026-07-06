"""Level 3 (characterization): the calibration-verify decision boundary.

``Gripper.verify_calibrated`` decides whether a saved encoder position is close
enough to the live position to skip a physical rehome. The refactor must not
move this boundary. The json round-trip / "avoid a rehome when a valid saved
value exists" path is exercised against ``ARGripperStandalone`` in
test_standalone.py; here we pin the pure decision on the unchanged ``Gripper``.
"""

import pytest

SID = 1
DEFAULT_MARGIN = 200


def _verify(make_gripper, present, previous, **kwargs):
    gripper, fake = make_gripper(servo_id=SID)
    fake.servo(SID).present_position_value = present
    return gripper.verify_calibrated(previous, **kwargs)


@pytest.mark.parametrize(
    "present, previous, expected",
    [
        (2000, 2000, True),  # identical
        (2000, 2200, True),  # +margin exactly (inclusive)
        (2000, 1800, True),  # -margin exactly (inclusive)
        (2000, 2201, False),  # just past +margin
        (2000, 1799, False),  # just past -margin
        (2000, 4000, False),  # finger swap -> genuine rehome
    ],
)
def test_default_margin_boundary(make_gripper, present, previous, expected):
    assert _verify(make_gripper, present, previous) is expected


def test_default_margin_is_200(make_gripper):
    assert _verify(make_gripper, 1000, 1000 + DEFAULT_MARGIN) is True
    assert _verify(make_gripper, 1000, 1000 + DEFAULT_MARGIN + 1) is False


def test_explicit_margin_overrides_default(make_gripper):
    assert _verify(make_gripper, 1000, 1050, margin=50) is True
    assert _verify(make_gripper, 1000, 1051, margin=50) is False


def test_updates_calibrated_flag(make_gripper):
    gripper, fake = make_gripper(servo_id=SID)
    fake.servo(SID).present_position_value = 2000
    assert gripper.verify_calibrated(2000) is True
    assert gripper.calibrated is True
    fake.servo(SID).present_position_value = 4000
    assert gripper.verify_calibrated(2000) is False
    assert gripper.calibrated is False
