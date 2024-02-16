cmake_minimum_required(VERSION 3.8)
project(ar_gripper)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install resource files
install(DIRECTORY
  launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}/
)
ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
