#!/usr/bin/env python
import json
import sys
import threading

import rclpy
from control_msgs.action import GripperCommand
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_srvs.srv import Empty

STATUS_UPDATE_INTERVAL_S = 0.2
UPDATE_RATE_HZ = 20


class Gripper:
    STATUS_UPDATE_RATE_HZ = 50
    ACTION_TIMEOUT_S = 10.0

    def __init__(self, action_name, gripper_name, node, external_sim=False):
        self.name = gripper_name
        self._node = node
        self._position = 100.0
        self._at_pos = True
        self._external_sim = external_sim
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self._node,
            GripperCommand,
            action_name,
            execute_callback=self._gripper_action_execute,
            handle_accepted_callback=self._handle_accepted_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        # Queue of 1, latch the last message
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        cmd_pos_topic = f"~/{gripper_name}/cmd_pos"
        self._cmd_pos_pub = self._node.create_publisher(
            Float32,
            cmd_pos_topic,
            qos_profile=qos_profile,
        )
        cmd_effort_topic = f"~/{gripper_name}/cmd_effort"
        self._cmd_effort_pub = self._node.create_publisher(
            Float32,
            cmd_effort_topic,
            qos_profile=qos_profile,
        )

        self._subs = []
        fb_pos_topic = f"~/{gripper_name}/fb_pos"
        self._subs.append(
            self._node.create_subscription(
                Float32, fb_pos_topic, self._on_fb_pos_received, 10
            )
        )
        fb_closed_topic = f"~/{gripper_name}/fb_at_pos"
        self._subs.append(
            self._node.create_subscription(
                Float32, fb_closed_topic, self._on_fb_at_pos_received, 10
            )
        )

    @property
    def position(self):
        return self._position

    @property
    def at_pos(self):
        return self._at_pos

    def _handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self._node.get_logger().info("Aborting previous goal")
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def _goal_callback(self, _goal_request):
        self._node.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal):
        self._node.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def _gripper_action_execute(self, goal_handle):
        self._node.get_logger().info(
            "Execute goal: position=%.1f, max_effort=%.1f"
            % (
                goal_handle.request.command.position,
                goal_handle.request.command.max_effort,
            )
        )
        self._at_pos = self._position == goal_handle.request.command.position
        self._cmd_pos_pub.publish(Float32(data=goal_handle.request.command.position))
        self._cmd_effort_pub.publish(
            Float32(data=goal_handle.request.command.max_effort)
        )

        if self._external_sim:
            r = self._node.create_rate(self.STATUS_UPDATE_RATE_HZ)
            start_time = self._node.get_clock().now()
            reached = False
            while rclpy.ok():
                if not goal_handle.is_active:
                    self._node.get_logger().info("Goal aborted")
                    return GripperCommand.Result()

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self._node.get_logger().info("Goal canceled")
                    return GripperCommand.Result()
                if self._at_pos:
                    reached = True
                    break
                if self._node.get_clock().now() - start_time >= rclpy.time.Duration(
                    seconds=self.ACTION_TIMEOUT_S
                ):
                    break
                r.sleep()
        else:
            self._position = goal_handle.request.command.position
            reached = True

        with self._goal_lock:
            if not goal_handle.is_active:
                self._node.get_logger().info("Goal aborted")
                return GripperCommand.Result()

            goal_handle.succeed()
            result = GripperCommand.Result()
            # not necessarily the current position of the gripper
            # if the gripper did not reach its goal position.
            result.position = goal_handle.request.command.position
            result.effort = goal_handle.request.command.max_effort
            result.stalled = False
            result.reached_goal = reached
            return result

    def _on_fb_pos_received(self, msg):
        self._position = msg.data

    def _on_fb_at_pos_received(self, msg):
        self._at_pos = msg.data

    def _on_connected(self):
        self._node.get_logger().info("Simulation connected")
        self._connected = True

    def _on_disconnected(self):
        self._node.get_logger().info("Simulation disconnected")
        self._connected = False


class GripperStatus:
    FINGER_OPEN_POS = -0.05
    FINGER_CLOSED_POS = 0.0

    def __init__(self, grippers, node):
        self._node = node
        self._grippers = grippers
        self._pub = self._node.create_publisher(JointState, "joint_states", 5)
        self._pos_fb_pubs = {}
        for gripper in (g.gripper for g in self._grippers):
            self._pos_fb_pubs[gripper.name] = self._node.create_publisher(
                JointState, f"~/{gripper.name}/position_feedback", 5
            )

    def publish_status(self):
        state_msg = JointState()
        state_msg.header.stamp = self._node.get_clock().now().to_msg()
        for gripper in (g.gripper for g in self._grippers):
            pos = gripper.position
            joint_pos = self.FINGER_OPEN_POS + (100.0 - pos) / 100.0 * (
                self.FINGER_CLOSED_POS - self.FINGER_OPEN_POS
            )
            state_msg.name.append(f"{gripper.name}_ar_gripper_body_finger1")
            state_msg.position.append(joint_pos)

            pos_msg = JointState()
            pos_msg.header.stamp = state_msg.header.stamp
            pos_msg.name.append("finger1")
            pos_msg.position.append(pos)
            self._pos_fb_pubs[gripper.name].publish(pos_msg)

        self._pub.publish(state_msg)


class ARGripper:
    def __init__(self, gripper_name, node, external_sim=False):
        self._node = node
        self.gripper = Gripper(f"~/{gripper_name}", gripper_name, node, external_sim)
        self._calibrate_srv = self._node.create_service(
            Empty, f"~/{gripper_name}/calibrate", self._calibrate_srv
        )

    def _calibrate_srv(self, _request, response):
        self._node.get_logger().info("Calibrate service called")
        return response


class ARGripperSimNode(Node):
    STATUS_UPDATE_INTERVAL_S = 0.2

    def __init__(self):
        super().__init__("ar_gripper_sim")

        external_sim = self.declare_parameter(
            "external_sim",
            False,
            ParameterDescriptor(
                description=(
                    "Whether the gripper is connected to an external simulation, such as Gazebo, or "
                    "if this node should provide the output"
                ),
                read_only=True,
            ),
        )
        gripper_params_json = self.declare_parameter(
            "grippers",
            "{}",
            ParameterDescriptor(
                description="The parameters for each gripper. See additional_constraints for format",
                additional_constraints=(
                    'Must be a JSON string in the format {"gripper_1_name_string": '
                    '[1], "gripper_2_name": [5]}, where the value is the servo RS-485 ID. '
                    "Currently only one servo ID per gripper is supported"
                ),
                read_only=True,
            ),
        )
        gripper_params = json.loads(gripper_params_json.value)

        self.get_logger().info("ARGripper sim driver starting")

        grippers = []
        for gripper_name, servo_ids in gripper_params.items():
            if len(servo_ids) != 1:
                raise ValueError(
                    f"Only one servo ID per gripper is supported, but gripper '{gripper_name}' has {len(servo_ids)}"
                )
            gripper = ARGripper(gripper_name, self, external_sim=external_sim.value)
            grippers.append(gripper)

        self.status = GripperStatus(grippers, self)
        self.create_timer(self.STATUS_UPDATE_INTERVAL_S, self._send_status)

    def _send_status(self):
        try:
            self.status.publish_status()
        except Exception as e:
            self.get_logger().error(
                f"Exception while publishing status {e}", throttle_duration_sec=5.0
            )


def main():
    print("DEPRECATED! No longer matches ar_gripper.py.")
    sys.exit(1)

    rclpy.init(args=sys.argv)

    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        ar_gripper_node = ARGripperSimNode()
        executor.add_node(ar_gripper_node)

        try:
            executor.spin()
        finally:
            ar_gripper_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
