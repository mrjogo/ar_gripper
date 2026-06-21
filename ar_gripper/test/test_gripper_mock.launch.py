# Standalone test of the mock (mock_components/GenericSystem) gripper ros2_control.
#
# Brings up a gripper-only mock robot_description (ar_gripper_standalone_mock.urdf.xacro)
# on a real controller_manager with the ar_gripper GripperActionController, then sends a
# GripperCommand and asserts the controller reaches the commanded position. Because mock
# hardware mirrors command to state with no physics, reaching the goal proves the mock
# ros2_control + controller wiring works without Gazebo.
#
# The controller advertises <controller>/gripper_cmd; it is remapped to
# /ar_gripper/primary/gripper_cmd (the namespace MoveIt uses, per
# config/moveit_controllers.yaml) via the spawner's --controller-ros-args, so this also
# covers that remap.

import unittest

import launch_testing
import launch_testing.actions
import rclpy
from control_msgs.action import GripperCommand
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from rclpy.action import ActionClient

import launch
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

GRIPPER_ACTION = "/ar_gripper/primary/gripper_cmd"
GRIPPER_TARGET = 0.0  # m; close from the 0.03 start
POSITION_TOL = 0.05


def generate_test_description():
    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ar_gripper"),
                    "urdf",
                    "ar_gripper_standalone_mock.urdf.xacro",
                ]
            ),
        ]
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)}
        ],
    )

    controllers_file = PathJoinSubstitution(
        [FindPackageShare("ar_gripper"), "config", "ar_gripper_sim_controllers.yaml"]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ParameterFile(controllers_file, allow_substs=True)],
        remappings=[("~/robot_description", "robot_description")],
        output="screen",
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ar_gripper",
            "-c",
            "/controller_manager",
            "--controller-ros-args",
            "-r /ar_gripper/gripper_cmd:=/ar_gripper/primary/gripper_cmd",
        ],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            robot_state_publisher,
            control_node,
            gripper_spawner,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestGripperMock(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_gripper_mock_client")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_gripper_command_executes(self):
        client = ActionClient(self.node, GripperCommand, GRIPPER_ACTION)
        self.assertTrue(
            client.wait_for_server(timeout_sec=60.0),
            f"Gripper action server {GRIPPER_ACTION} not available",
        )

        goal = GripperCommand.Goal()
        goal.command.position = GRIPPER_TARGET
        goal.command.max_effort = 50.0

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=15.0)
        goal_handle = send_future.result()
        self.assertIsNotNone(goal_handle, "send_goal returned no handle")
        self.assertTrue(goal_handle.accepted, "Gripper goal was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=15.0)
        wrapped = result_future.result()
        self.assertIsNotNone(wrapped, "No result returned for gripper goal")

        result = wrapped.result
        self.assertTrue(result.reached_goal, "Gripper did not report reached_goal")
        self.assertAlmostEqual(result.position, GRIPPER_TARGET, delta=POSITION_TOL)
