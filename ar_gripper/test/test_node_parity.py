"""Level 4: the ROS node behaves identically before and after the refactor.

Builds the real ``ARGripperNode`` in-process (params via a file, FakeSerial under
it) and drives its methods directly — no executor, no real bus. Asserts the
GripperCommand result fields, /joint_states values, and diagnostics equal the
golden captured from the pre-refactor node. These are characterization tests:
they pass on ``main`` and must stay green through the shim refactor.

Skipped automatically where rclpy is unavailable (e.g. a bare LeRobot venv).
"""

import pytest

rclpy = pytest.importorskip("rclpy")

SID = 1
JOINT_NAME = "primary_ar_gripper_body_finger1"


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


@pytest.fixture
def ros_node(fake_serials, fast_clock, tmp_path):
    """A real ARGripperNode wired to FakeSerial. Yields (node, fake, nodemod)."""
    import ar_gripper.scripts.ar_gripper as nodemod

    saved = tmp_path / "servo_position.json"
    saved.write_text('{"position": 150}')  # matches FakeServo default -> no rehome
    params = tmp_path / "params.yaml"
    params.write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    port: /dev/fake\n"
        '    baud: "115200"\n'
        "    grippers: '{\"primary\": [1]}'\n"
        f"    servo_position_path: {saved}\n"
    )
    rclpy.init(args=["--ros-args", "--params-file", str(params)])
    node = nodemod.ARGripperNode()
    try:
        yield node, fake_serials[-1], nodemod
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _gripper(node):
    return node._grippers[0]


def test_node_builds_single_gripper(ros_node):
    node, _fake, _nodemod = ros_node
    assert [g.gripper.name for g in node._grippers] == ["primary"]
    assert _gripper(node).gripper.calibrated is True


def test_action_goto_result_fields(ros_node):
    node, _fake, nodemod = ros_node
    from control_msgs.action import GripperCommand

    handle = _FakeGoalHandle(0.0, 500.0, GripperCommand)
    result = _gripper(node)._gripper_action_execute(handle)
    assert result.position == 0.0
    assert result.effort == 0.0
    assert result.reached_goal is True
    assert result.stalled is False
    assert handle.succeeded is True


def test_action_release_result_fields(ros_node):
    node, fake, _nodemod = ros_node
    from control_msgs.action import GripperCommand

    fake.servo(SID).present_position_value = 2122  # 50% -> 0.025 m
    handle = _FakeGoalHandle(0.03, 0.0, GripperCommand)  # max_effort 0 -> release
    result = _gripper(node)._gripper_action_execute(handle)
    assert result.position == pytest.approx(0.025)
    assert result.effort == 0.0
    assert result.reached_goal is False  # 0.025 != requested 0.03
    assert result.stalled is False


def test_joint_states_publish(ros_node):
    node, fake, _nodemod = ros_node
    fake.servo(SID).present_position_value = 2122  # 50% -> 0.025 m
    captured = []
    node._joint_state_pub.publish = lambda msg: captured.append(msg)
    node._send_status()
    msg = captured[-1]
    assert list(msg.name) == [JOINT_NAME]
    assert list(msg.position) == pytest.approx([0.025])


def test_diagnostics_ok(ros_node):
    node, _fake, _nodemod = ros_node
    from diagnostic_msgs.msg import DiagnosticStatus

    captured = []
    node._diagnostics_pub.publish = lambda msg: captured.append(msg)
    node._send_diagnostics()
    status = captured[-1].status[0]
    assert status.name == "Gripper 'primary' servo 1"
    assert status.level == DiagnosticStatus.OK
    assert status.message == "OK"
    values = {kv.key: kv.value for kv in status.values}
    assert values == {"Temperature": "25", "Voltage": "12.0"}


def test_diagnostics_overheating(ros_node):
    node, fake, _nodemod = ros_node
    from diagnostic_msgs.msg import DiagnosticStatus

    fake.servo(SID).reg[0x3F] = 75  # present_temperature
    captured = []
    node._diagnostics_pub.publish = lambda msg: captured.append(msg)
    node._send_diagnostics()
    status = captured[-1].status[0]
    assert status.level == DiagnosticStatus.ERROR
    assert status.message == "OVERHEATING"


def _write_params(tmp_path, grippers, saved_position_path):
    params = tmp_path / "params.yaml"
    params.write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    port: /dev/fake\n"
        '    baud: "115200"\n'
        f"    grippers: '{grippers}'\n"
        f"    servo_position_path: {saved_position_path}\n"
    )
    return params


def test_calibration_failure_exits_process(fake_serials, fast_clock, tmp_path):
    """A failed startup rehome exits the process with the original message.

    The FakeServo's default present_load (0.0) never reaches the homing contact
    threshold, so with no saved file to reuse, calibration fails; the shim must
    translate ARGripperStandalone's CalibrationError into the same sys.exit the
    pre-shim node used.
    """
    import ar_gripper.scripts.ar_gripper as nodemod

    missing = tmp_path / "does_not_exist.json"  # forces a rehome
    params = _write_params(tmp_path, '{"primary": [1]}', missing)
    rclpy.init(args=["--ros-args", "--params-file", str(params)])
    try:
        with pytest.raises(SystemExit) as excinfo:
            nodemod.ARGripperNode()
        assert excinfo.value.code == "Gripper calibration failed"
    finally:
        rclpy.shutdown()


def test_multiple_grippers_share_one_serial_bus(fake_serials, fast_clock, tmp_path):
    """Two grippers on one bus open exactly one USB2FeetechDevice (shared)."""
    import ar_gripper.scripts.ar_gripper as nodemod

    saved = tmp_path / "servo_position.json"
    saved.write_text('{"position": 150}')  # matches FakeServo default -> no rehome
    params = _write_params(tmp_path, '{"a": [1], "b": [2]}', saved)
    rclpy.init(args=["--ros-args", "--params-file", str(params)])
    node = None
    try:
        node = nodemod.ARGripperNode()
        assert len(fake_serials) == 1  # single shared serial device
        assert len(node._grippers) == 2
        assert len(node.all_servos) == 2
        assert sorted(g.gripper.name for g in node._grippers) == ["a", "b"]
        # Both servos (ids 1 and 2) round-trip over the one bus.
        captured = []
        node._diagnostics_pub.publish = lambda msg: captured.append(msg)
        node._send_diagnostics()
        assert len(captured[-1].status) == 2
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
