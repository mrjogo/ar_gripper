"""The launchable mock mode: the real ARGripperNode runs on an in-process fake bus.

Unlike test_node_parity (which installs the FakeSerial via the pytest monkeypatch
fixture), this exercises the *production* seam: constructing ARGripperNode with the
``mock`` parameter set makes the node install the loopback itself, so
``ros2 launch ar_gripper ar_gripper_control.launch.py mock:=true`` brings the real
driver up with no hardware and no test harness. Asserts the node homes through the
real calibration routine and a grasp round-trips over the fake bus.

Skipped automatically where rclpy is unavailable (e.g. a bare LeRobot venv).
"""

import pytest

rclpy = pytest.importorskip("rclpy")


class _FakeGoalHandle:
    """Stand-in for an rclpy action goal handle (bypasses the action server)."""

    def __init__(self, position, max_effort, GripperCommand):
        self.request = GripperCommand.Goal()
        self.request.command.position = position
        self.request.command.max_effort = max_effort
        self.is_active = True
        self.is_cancel_requested = False
        self.succeeded = False
        self.canceled_flag = False

    def succeed(self):
        self.succeeded = True

    def canceled(self):
        self.canceled_flag = True


def _write_params(tmp_path):
    params = tmp_path / "params.yaml"
    params.write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    port: /dev/fake\n"
        '    baud: "115200"\n'
        "    grippers: '{\"primary\": [1]}'\n"
        "    mock: true\n"
    )
    return params


@pytest.fixture
def restore_mock_patches():
    """Undo the global serial/clock patches install_ros_node_loopback leaves behind.

    The node stays mocked for its lifetime (no restore), so a test must put the real
    serial factory / clock back or it would leak into other tests.
    """
    from ar_gripper import feetech, gripper

    orig_serial = feetech.serial.serial_for_url
    orig_time = gripper.time
    yield
    feetech.serial.serial_for_url = orig_serial
    gripper.time = orig_time


def test_node_homes_and_grasps_in_mock_mode(tmp_path, restore_mock_patches):
    from control_msgs.action import GripperCommand

    import ar_gripper.scripts.ar_gripper as nodemod

    rclpy.init(args=["--ros-args", "--params-file", str(_write_params(tmp_path))])
    node = None
    try:
        # The node installs its own fake bus from the `mock` param -- no fixture
        # patched serial here -- then homes through the real Gripper.calibrate().
        node = nodemod.ARGripperNode()
        gripper = node._grippers[0]
        assert gripper.gripper.name == "primary"
        assert gripper.gripper.calibrated is True

        # A close command round-trips over the fake bus through the real
        # action-execute path and reports the reached position.
        handle = _FakeGoalHandle(0.0, 500.0, GripperCommand)
        result = gripper._gripper_action_execute(handle)
        assert result.position == pytest.approx(0.0, abs=1e-3)
        assert result.reached_goal is True
        assert handle.succeeded is True
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
