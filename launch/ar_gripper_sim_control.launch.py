from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ar_gripper", "-c", "/controller_manager"],
    )

    # Launching Gazebo and robot spawning must be done separately
    return [
        gripper_controller_spawner,
    ]


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
