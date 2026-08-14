"""Regression test for the bootstrap-executor ordering bug a live Isaac run
found: startup calibration runs synchronously inside ``ARGripperNode.__init__``,
before ``main()``'s executor ever spins the node, so without something
spinning it in the background ``IsaacJointBus``'s subscription and ramp timer
never run and any blocking read starves forever waiting for a sample that can
never arrive.

Reproduces ``ar_gripper.py``'s isaac branch's own bootstrap pattern directly
(a private ``SingleThreadedExecutor`` spun on a background thread while the
test's main thread blocks on ``IsaacJointBus.wait_for_sample``, exactly as
``Gripper._calibrate()`` blocks inside ``__init__``) without going through the
whole node / calibration machinery, which would need a scripted hard-stop
responder to converge and would make this a much larger, slower test for the
same guarantee. Needs a real rclpy node (unlike ``test_isaac_servo.py``),
skipped automatically where rclpy is unavailable.
"""

import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState

from ar_gripper.scripts.isaac_servo import IsaacJointBus

JOINT_NAME = "primary_ar_gripper_body_finger1"


def test_wait_for_sample_is_serviced_by_a_private_background_executor():
    """The exact shape that starved in the live bug: a synchronous, blocking
    read on the calling thread, with nothing but a private executor spinning
    the node in the background to service the subscription that would
    satisfy it.
    """
    rclpy.init()
    try:
        node = rclpy.create_node("test_isaac_bootstrap_node")
        publisher_node = rclpy.create_node("test_isaac_bootstrap_publisher")
        pub = publisher_node.create_publisher(JointState, "/isaac/joint_states", 10)

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(publisher_node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        try:
            bus = IsaacJointBus(
                node,
                joint_states_topic="/isaac/joint_states",
                command_topic="/isaac/gripper/joint_commands",
                joint_name=JOINT_NAME,
            )

            stop = threading.Event()

            def _publish_loop():
                msg = JointState()
                msg.name = [JOINT_NAME]
                msg.position = [0.01]
                msg.velocity = [0.0]
                msg.effort = [0.0]
                while not stop.is_set():
                    pub.publish(msg)
                    time.sleep(0.01)

            pub_thread = threading.Thread(target=_publish_loop, daemon=True)
            pub_thread.start()
            try:
                position_m, _velocity_mps, _effort_n = bus.wait_for_sample(
                    timeout_s=2.0
                )
            finally:
                stop.set()
                pub_thread.join(timeout=2.0)

            assert position_m == pytest.approx(0.01)
            # A nonzero count here would mean the subscription never got
            # serviced and wait_for_sample only returned via its own
            # timeout fallback -- exactly the bug this test guards against.
            assert bus.timeout_count == 0
        finally:
            executor.remove_node(node)
            executor.remove_node(publisher_node)
            executor.shutdown()
            spin_thread.join(timeout=2.0)
            node.destroy_node()
            publisher_node.destroy_node()
    finally:
        rclpy.shutdown()
