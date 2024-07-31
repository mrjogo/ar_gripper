from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    grippers = LaunchConfiguration("grippers")

    ar_gripper_node = Node(
        package="ar_gripper",
        executable="ar_gripper",
        parameters=[
            {
                "port": port,
                "baud": ParameterValue(baud, value_type=str),
                "grippers": ParameterValue(grippers, value_type=str),
            },
        ],
        output="screen",
    )

    return [ar_gripper_node]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "port",
            default_value="hwgrep://1a86:7523",
            description="Port to communicate with the gripper",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "baud",
            default_value="115200",
            description="Baud rate for the gripper communication",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "grippers",
            default_value='{ "primary": [0] }',
            description=(
                "JSON string with gripper name, array of RS-485 IDs (currently only "
                "one per gripper supported)"
            ),
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
