from control_msgs.action import GripperCommand
from std_srvs.srv import Empty
from rclpy.action import ActionClient


class ARGripper:
    GRIP_MAX = 100.0
    GRIP_MIN = 0.0

    def __init__(self, name, node):
        self.name = name
        self._node = node
        self._grip_value = self.GRIP_MAX
        self._grip_step = self.GRIP_MAX / 15  # gripper step Cross Up and Cross Down
        self._connect_to_gripper_action()
        self._connect_to_calibrate_srv()

    def _connect_to_gripper_action(self):
        self._node.get_logger().info(f"Waiting for action server {self.name}...")
        self._client = ActionClient(self._node, GripperCommand, self.name)
        self._client.wait_for_server(timeout_sec=60)
        self._node.get_logger().info("Connected to action server")

    def _connect_to_calibrate_srv(self):
        service_name = self.name + "/calibrate"
        self._node.get_logger().info(f"Waiting for service {service_name}...")
        self._calibrate_srv = self._node.create_client(Empty, service_name)
        self._calibrate_srv.wait_for_service()
        self._node.get_logger().info(f"Connected to service {service_name}")

    def calibrate(self):
        self._node.get_logger().info("argripper_interface: calibrate")
        self._calibrate_srv.call(Empty.Request())
        self._grip_value = self.GRIP_MAX
        self._node.get_logger().info("argripper_interface: calibrate done")

    def open_step(self):
        self._grip_value = self._grip_value + self._grip_step
        self._grip_value = min(self._grip_value, self.GRIP_MAX)
        self._node.get_logger().info(
            f"argripper_interface: goto position {self._grip_value:.3f}"
        )
        goal = GripperCommand.Goal()
        goal.command.position = self._grip_value
        goal.command.max_effort = 50.0
        self._client.send_goal(goal)
        self._node.get_logger().info("argripper_interface: goto position done")

    def close_step(self):
        self._grip_value = self._grip_value - self._grip_step
        self._grip_value = max(self._grip_value, self.GRIP_MIN)
        self._node.get_logger().info(
            f"argripper_interface: goto position {self._grip_value:.3f}"
        )
        goal = GripperCommand.Goal()
        goal.command.position = self._grip_value
        goal.command.max_effort = 50.0
        self._client.send_goal(goal)
        self._node.get_logger().info("argripper_interface: goto position done")

    def close(self, max_effort):
        self._node.get_logger().info(
            f"argripper_interface: close, effort {max_effort:.1f}"
        )
        goal = GripperCommand.Goal()
        goal.command.position = 0.0
        goal.command.max_effort = max_effort
        self._client.send_goal(goal)
        self._node.get_logger().info("argripper_interface: close done")
        self._grip_value = self.GRIP_MIN

    def hard_close(self):
        self._node.get_logger().info("argripper_interface: hard close")
        goal = GripperCommand.Goal()
        goal.command.position = 0.0
        goal.command.max_effort = 100.0  # >0 to 100
        self._client.send_goal(goal)
        self._node.get_logger().info("argripper_interface: hard close done")
        self._grip_value = self.GRIP_MIN

    def soft_close(self):
        self._node.get_logger().info("argripper_interface: soft close")
        goal = GripperCommand.Goal()
        goal.command.position = 0.0
        goal.command.max_effort = 20.0  # >0 to 100
        self._client.send_goal(goal)
        self._node.get_logger().info("argripper_interface: soft close done")
        self._grip_value = self.GRIP_MIN

    def open(self):
        self._node.get_logger().info("argripper_interface: open")
        goal = GripperCommand.Goal()
        goal.command.position = 100.0  # 100% range (0..100)
        goal.command.max_effort = 100.0  # >0 to 100
        self._client.send_goal(goal)
        self._node.get_logger().info("argripper_interface: open done")
        self._grip_value = self.GRIP_MAX

    def goto_position(self, grip_position=5.0, grip_effort=20.0):
        # position in % 0 to 100 (0 is closed), effort in % 0 to 100
        self._node.get_logger().info(
            f"argripper_interface: goto position {grip_position:.3f}"
        )
        goal = GripperCommand.Goal()
        goal.command.position = grip_position  # range(0.0 to 100.0)
        goal.command.max_effort = grip_effort  # > 0 to 100, if 0.0, torque is released
        self._client.send_goal(goal)
        self._node.get_logger().info("argripper_interface: goto position done")
        self._grip_value = grip_position

    def release(self):
        self._node.get_logger().info("argripper_interface: release")
        goal = GripperCommand.Goal()
        goal.command.position = 0.0  # not dependent on position
        goal.command.max_effort = 0.0  # max_effort = 0.0 releases all torque on motor
        self._client.send_goal(goal)
        self._node.get_logger().info("argripper_interface: release done")
        self._grip_value = self.GRIP_MIN
