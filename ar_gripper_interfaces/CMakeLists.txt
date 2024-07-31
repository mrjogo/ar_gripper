cmake_minimum_required(VERSION 3.8)
project(ar_gripper_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Build ROS messages, services and actions
set(srv_files
  "srv/SetHoldingTorque.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)

ament_package()
