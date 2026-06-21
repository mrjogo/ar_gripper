cmake_minimum_required(VERSION 3.8)
project(ar_gripper)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install resource files
install(DIRECTORY
  launch meshes rviz urdf srdf config
  DESTINATION share/${PROJECT_NAME}/
)
ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Mock gripper ros2_control test: GripperCommand onto a standalone mock
  # controller_manager, asserting the controller reaches the commanded position.
  add_launch_test(test/test_gripper_mock.launch.py
    TIMEOUT 120
  )
endif()

ament_package()
