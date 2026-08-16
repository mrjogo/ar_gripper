from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    grippers = LaunchConfiguration("grippers")
    mock = LaunchConfiguration("mock")
    isaac = LaunchConfiguration("isaac")
    isaac_joint_states_topic = LaunchConfiguration("isaac_joint_states_topic")
    isaac_command_topic = LaunchConfiguration("isaac_command_topic")
    bus_trace_path = LaunchConfiguration("bus_trace_path")
    bus_trace_sample_hz = LaunchConfiguration("bus_trace_sample_hz")

    ar_gripper_node = Node(
        package="ar_gripper",
        executable="ar_gripper",
        parameters=[
            {
                "port": port,
                "baud": ParameterValue(baud, value_type=str),
                "grippers": ParameterValue(grippers, value_type=str),
                "mock": ParameterValue(mock, value_type=bool),
                "isaac": ParameterValue(isaac, value_type=bool),
                "isaac_joint_states_topic": ParameterValue(
                    isaac_joint_states_topic, value_type=str
                ),
                "isaac_command_topic": ParameterValue(
                    isaac_command_topic, value_type=str
                ),
                "bus_trace_path": ParameterValue(bus_trace_path, value_type=str),
                "bus_trace_sample_hz": ParameterValue(
                    bus_trace_sample_hz, value_type=float
                ),
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock",
            default_value="false",
            description=(
                "Run the driver against an in-process fake Feetech bus instead of "
                "real serial hardware (hardware-free bring-up; no serial port needed)."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "isaac",
            default_value="false",
            description=(
                "Run the driver against Isaac Sim's simulated finger joint instead "
                "of real serial hardware or the offline mock. Mutually exclusive "
                "with mock."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "isaac_joint_states_topic",
            default_value="/isaac/joint_states",
            description="Isaac joint-state topic the servo backend reads.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "isaac_command_topic",
            default_value="/isaac/gripper/joint_commands",
            description="Isaac joint-command topic the servo backend publishes.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "bus_trace_path",
            default_value="",
            description=(
                "Write a timestamped trace of every Feetech bus transaction here "
                "on shutdown, for motion profiling. Empty (default) disables it "
                "and leaves the bus path untouched."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bus_trace_sample_hz",
            default_value="0.0",
            description=(
                "With bus_trace_path set, also read present_position at this rate. "
                "0 (default) does not. This ADDS bus traffic and competes with the "
                "control loop; prefer the passive trace unless you need the "
                "resolution."
            ),
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
