cmake_minimum_required(VERSION 3.8)
project(ar_gripper_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# For std_msgs/Header on GripperState
find_package(std_msgs REQUIRED)

# Build ROS messages, services and actions
set(msg_files
  "msg/GripperState.msg"
)

set(srv_files
  "srv/SetHoldingTorque.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

ament_package()
