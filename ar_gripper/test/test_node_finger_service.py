"""Finger-service mode on the ROS node: nothing moves, nothing grasps.

The operator is standing at the robot with a jammed gripper. What this mode has
to guarantee, in order: that bringing the driver up does not move anything (the
fingers are what is wrong, and the startup rehome would drive them into the
thing that jammed); that nothing else can command the gripper while their hands
are in it; and that the two service motions are reachable and leave the fingers
slack when they end.

The real ``ARGripperNode`` is built in-process over a FakeSerial and its
callbacks are driven directly -- the same harness test_node_parity.py uses. The
motions themselves are pinned at the ``Gripper`` level in test_finger_service.py.
"""

import time as real_time

import pytest

rclpy = pytest.importorskip("rclpy")

SID = 1
GOAL_POSITION = 0x2A
TORQUE_SWITCH = 0x28
RESET_MIDPOINT = 128
TORQUE_OFF = 0

# The names an operator types into `ros2 service call`, which is the thing
# worth pinning: rclpy keeps the unexpanded "~/..." on the service object, so
# the test resolves it rather than asserting on the private form.
OPEN_SERVICE = "/ar_gripper/primary/finger_service_open"
CLOSE_SERVICE = "/ar_gripper/primary/finger_service_close"


def _writes(fake, servo_id=SID):
    out = []
    for packet in fake.trace:
        if len(packet) < 6 or packet[0] != 0xFF or packet[1] != 0xFF:
            continue
        if packet[2] != servo_id:
            continue
        instruction = packet[4 : 4 + (packet[3] - 1)]
        if not instruction or instruction[0] != 0x03:
            continue
        out.append((instruction[1], list(instruction[2:])))
    return out


def _moved(fake):
    """Any write that could have turned the motor."""
    return [
        (addr, data)
        for addr, data in _writes(fake)
        if addr == GOAL_POSITION or (addr == TORQUE_SWITCH and data == [RESET_MIDPOINT])
    ]


def _wait_for(predicate, timeout_s=10.0):
    deadline = real_time.monotonic() + timeout_s
    while real_time.monotonic() < deadline:
        if predicate():
            return True
        real_time.sleep(0.005)
    return False


def _write_params(tmp_path, saved_position_path, finger_service):
    params = tmp_path / "params.yaml"
    params.write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    port: /dev/fake\n"
        '    baud: "115200"\n'
        "    grippers: '{\"primary\": [1]}'\n"
        f"    servo_position_path: {saved_position_path}\n"
        f"    finger_service: {'true' if finger_service else 'false'}\n"
    )
    return params


@pytest.fixture
def make_node(fake_serials, fast_clock, tmp_path):
    """Build a real ARGripperNode over FakeSerial. Yields a factory."""
    import ar_gripper.scripts.ar_gripper as nodemod

    built = []

    def _make(finger_service=True, saved_position=None):
        saved = tmp_path / "servo_position.json"
        if saved_position is not None:
            saved.write_text(f'{{"position": {saved_position}}}')
        params = _write_params(tmp_path, saved, finger_service)
        rclpy.init(args=["--ros-args", "--params-file", str(params)])
        node = nodemod.ARGripperNode()
        built.append(node)
        return node, fake_serials[-1], saved

    # A test that tears a node down itself drops it from here, so teardown does
    # not destroy it twice.
    _make.built = built

    yield _make
    for node in built:
        node._grippers[0].gripper.finger_service_open_stop()
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _gripper(node):
    return node._grippers[0]


def _service_names(node):
    return {node.resolve_service_name(service.srv_name) for service in node.services}


# --------------------------------------------------------------------------- #
# Bring-up: nothing moves
# --------------------------------------------------------------------------- #
def test_finger_service_mode_moves_nothing_at_startup(make_node):
    node, fake, _saved = make_node(finger_service=True)

    assert _moved(fake) == []
    assert _gripper(node).gripper.calibrated is False


def test_finger_service_mode_does_not_read_the_saved_position(make_node):
    """A saved position that would ordinarily skip the rehome is not consulted."""
    node, fake, _saved = make_node(finger_service=True, saved_position=150)

    assert _moved(fake) == []
    assert _gripper(node).gripper.calibrated is False


def test_without_the_parameter_startup_is_unchanged(make_node):
    node, _fake, _saved = make_node(finger_service=False, saved_position=150)

    assert _gripper(node).gripper.calibrated is True
    assert _service_names(node).isdisjoint({OPEN_SERVICE, CLOSE_SERVICE})


def test_the_two_services_exist_only_in_finger_service_mode(make_node):
    node, _fake, _saved = make_node(finger_service=True)

    assert {OPEN_SERVICE, CLOSE_SERVICE} <= _service_names(node)


# --------------------------------------------------------------------------- #
# Refusals
# --------------------------------------------------------------------------- #
def test_grasp_goals_are_refused(make_node):
    from control_msgs.action import GripperCommand
    from rclpy.action import GoalResponse

    node, _fake, _saved = make_node(finger_service=True)

    assert _gripper(node)._goal_callback(GripperCommand.Goal()) == GoalResponse.REJECT


def test_the_calibrate_service_is_refused(make_node):
    from std_srvs.srv import Empty

    node, fake, _saved = make_node(finger_service=True)

    _gripper(node)._handle_calibrate_srv(Empty.Request(), Empty.Response())

    assert _moved(fake) == []
    assert _gripper(node).gripper.calibrated is False


def test_the_refusal_says_how_to_get_out_of_the_mode():
    import ar_gripper.scripts.ar_gripper as nodemod

    assert nodemod.FINGER_SERVICE_REFUSAL == (
        "finger-service mode: restart without finger_service to grasp/calibrate"
    )


# --------------------------------------------------------------------------- #
# The two motions, over the node's own callbacks
# --------------------------------------------------------------------------- #
def test_open_true_then_false_leaves_the_fingers_slack(make_node, frozen_clock):
    from std_srvs.srv import SetBool

    node, fake, _saved = make_node(finger_service=True)
    gripper = _gripper(node)

    start = gripper._handle_finger_service_open(
        _set_bool(SetBool, True), SetBool.Response()
    )
    assert start.success is True
    assert _wait_for(lambda: _moved(fake))
    assert gripper.gripper.finger_service_open_active is True

    stop = gripper._handle_finger_service_open(
        _set_bool(SetBool, False), SetBool.Response()
    )
    assert stop.success is True
    assert gripper.gripper.finger_service_open_active is False
    assert _writes(fake)[-1] == (TORQUE_SWITCH, [TORQUE_OFF])
    assert stop.message


def test_close_returns_promptly_with_the_fingers_slack(make_node):
    """Nothing is driven, so this is a bus write and a read, not a wait."""
    from std_srvs.srv import Trigger

    node, fake, _saved = make_node(finger_service=True)
    gripper = _gripper(node)
    fake.trace.clear()  # drop bring-up writes from node/gripper construction

    started = real_time.monotonic()
    response = gripper._handle_finger_service_close(
        Trigger.Request(), Trigger.Response()
    )
    elapsed = real_time.monotonic() - started

    assert response.success is True
    assert response.message
    assert elapsed < 0.5
    assert _moved(fake) == []  # no goal, no re-reference -- the pinion is free
    assert _writes(fake) == [(TORQUE_SWITCH, [TORQUE_OFF])]


def test_close_is_refused_while_the_open_drive_is_running(make_node, frozen_clock):
    from std_srvs.srv import SetBool, Trigger

    node, fake, _saved = make_node(finger_service=True)
    gripper = _gripper(node)

    gripper._handle_finger_service_open(_set_bool(SetBool, True), SetBool.Response())
    assert _wait_for(lambda: _moved(fake))

    response = gripper._handle_finger_service_close(
        Trigger.Request(), Trigger.Response()
    )

    assert response.success is False
    gripper._handle_finger_service_open(_set_bool(SetBool, False), SetBool.Response())


def test_shutting_the_node_down_stops_a_drive_that_is_still_pushing(
    make_node, frozen_clock
):
    """Ctrl-C kills the daemon thread; the servo would never hear about it."""
    from std_srvs.srv import SetBool

    node, fake, _saved = make_node(finger_service=True)
    gripper = _gripper(node)

    gripper._handle_finger_service_open(_set_bool(SetBool, True), SetBool.Response())
    assert _wait_for(lambda: _moved(fake))

    make_node.built.remove(node)
    node.destroy_node()

    assert gripper.gripper.finger_service_open_active is False
    assert _writes(fake)[-1] == (TORQUE_SWITCH, [TORQUE_OFF])


def test_shutdown_cuts_torque_even_when_stopping_the_drive_fails(make_node):
    """The release must not depend on the stop having got as far as its own."""
    node, fake, _saved = make_node(finger_service=True)
    gripper = _gripper(node)

    def explode():
        raise RuntimeError("the bus went away")

    gripper.gripper.finger_service_open_stop = explode
    fake.trace.clear()

    make_node.built.remove(node)
    node.destroy_node()  # must not raise

    assert _writes(fake)[-1] == (TORQUE_SWITCH, [TORQUE_OFF])


def test_finger_service_never_writes_the_saved_position_file(make_node, frozen_clock):
    """The next startup has to fail verify_calibrated and rehome. A file stops that."""
    from std_srvs.srv import Empty, SetBool, Trigger

    node, fake, saved = make_node(finger_service=True)
    gripper = _gripper(node)

    gripper._handle_finger_service_open(_set_bool(SetBool, True), SetBool.Response())
    assert _wait_for(lambda: _moved(fake))
    gripper._handle_finger_service_open(_set_bool(SetBool, False), SetBool.Response())
    gripper._handle_finger_service_close(Trigger.Request(), Trigger.Response())
    gripper._handle_calibrate_srv(Empty.Request(), Empty.Response())

    assert not saved.exists()
    assert gripper._standalone._servo_position_path is None


def _set_bool(SetBool, value):
    request = SetBool.Request()
    request.data = value
    return request
