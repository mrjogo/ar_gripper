import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from std_srvs.srv import Empty


class ARGripper:
    GRIP_MAX = 100.0
    GRIP_MIN = 0.0

    def __init__(self, name):
        self.name = name
        self._grip_value = self.GRIP_MAX
        self._grip_step = (
            self.GRIP_MAX / 15
        )  # gripper step Cross Up and Cross Down
        self._connect_to_gripper_action()
        self._connect_to_calibrate_srv()

    def _connect_to_gripper_action(self):
        rospy.loginfo(f"Waiting for action server {self.name}...")
        self._client = actionlib.SimpleActionClient(
            self.name, GripperCommandAction
        )
        self._client.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to action server")

    def _connect_to_calibrate_srv(self):
        service_name = self.name + '/calibrate'
        rospy.loginfo(f"Waiting for service {service_name}...")
        rospy.wait_for_service(service_name)
        self._calibrate_srv = rospy.ServiceProxy(service_name, Empty)
        rospy.loginfo(f"Connected to service {service_name}")

    def calibrate(self):
        rospy.loginfo("argripper_interface: calibrate")
        try:
            self._calibrate_srv()
        except rospy.ServiceException as exc:
            rospy.logwarn(f"Service did not process request: {exc}")
        else:
            self._grip_value = self.GRIP_MAX
        rospy.loginfo("argripper_interface: calibrate done")

    def open_step(self):
        self._grip_value = self._grip_value + self._grip_step
        self._grip_value = min(self._grip_value, self.GRIP_MAX)
        rospy.loginfo(
            f"argripper_interface: goto position {self._grip_value:.3f}"
        )
        goal = GripperCommandGoal()
        goal.command.position = self._grip_value
        goal.command.max_effort = 50.0
        self._client.send_goal_and_wait(goal)
        rospy.loginfo("argripper_interface: goto position done")

    def close_step(self):
        self._grip_value = self._grip_value - self._grip_step
        self._grip_value = max(self._grip_value, self.GRIP_MIN)
        rospy.loginfo(
            f"argripper_interface: goto position {self._grip_value:.3f}"
        )
        goal = GripperCommandGoal()
        goal.command.position = self._grip_value
        goal.command.max_effort = 50.0
        self._client.send_goal_and_wait(goal)
        rospy.loginfo("argripper_interface: goto position done")

    def close(self, max_effort):
        rospy.loginfo(f"argripper_interface: close, effort {max_effort:.1f}")
        goal = GripperCommandGoal()
        goal.command.position = 0.0
        goal.command.max_effort = max_effort
        self._client.send_goal_and_wait(goal)
        rospy.loginfo("argripper_interface: close done")
        self._grip_value = self.GRIP_MIN

    def hard_close(self):
        rospy.loginfo("argripper_interface: hard close")
        goal = GripperCommandGoal()
        goal.command.position = 0.0
        goal.command.max_effort = 100  # >0 to 100
        self._client.send_goal_and_wait(goal)
        rospy.loginfo("argripper_interface: hard close done")
        self._grip_value = self.GRIP_MIN

    def soft_close(self):
        rospy.loginfo("argripper_interface: soft close")
        goal = GripperCommandGoal()
        goal.command.position = 0.0
        goal.command.max_effort = 20.0  # >0 to 100
        self._client.send_goal_and_wait(goal)
        rospy.loginfo("argripper_interface: soft close done")
        self._grip_value = self.GRIP_MIN

    def open(self):
        rospy.loginfo("argripper_interface: open")
        goal = GripperCommandGoal()
        goal.command.position = 100.0  # 100% range (0..100)
        goal.command.max_effort = 100.0  # >0 to 100
        self._client.send_goal_and_wait(goal)
        rospy.loginfo("argripper_interface: open done")
        self._grip_value = self.GRIP_MAX

    def goto_position(self, grip_position=5.0, grip_effort=20.0):
        # position in % 0 to 100 (0 is closed), effort in % 0 to 100
        rospy.loginfo(f"argripper_interface: goto position {grip_position:.3f}")
        goal = GripperCommandGoal()
        goal.command.position = grip_position  # range(0.0 to 100.0)
        goal.command.max_effort = (
            grip_effort  # > 0 to 100, if 0.0, torque is released
        )
        self._client.send_goal_and_wait(goal)
        rospy.loginfo("argripper_interface: goto position done")
        self._grip_value = grip_position

    def release(self):
        rospy.loginfo("argripper_interface: release")
        goal = GripperCommandGoal()
        goal.command.position = 0.0  # not dependent on position
        goal.command.max_effort = (
            0.0  # max_effort = 0.0 releases all torque on motor
        )
        self._client.send_goal_and_wait(goal)
        rospy.loginfo("argripper_interface: release done")
        self._grip_value = self.GRIP_MIN
