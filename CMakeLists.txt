cmake_minimum_required(VERSION 2.8.3)
project(ar_gripper)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS rospy)

# Install python from setup.py
catkin_python_setup()

#
# catkin specific configuration ##
#
# The catkin_package macro generates cmake config files for your package

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES gripper4_description
  #  CATKIN_DEPENDS other_catkin_pkg
  #  DEPENDS system_lib
)

#
# Install ##
#

catkin_install_python(
  PROGRAMS scripts/${PROJECT_NAME} scripts/${PROJECT_NAME}_sim
  scripts/${PROJECT_NAME}_gui DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#
# Testing ##
#

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(ros_pytest REQUIRED)
  #add_rostest(tests/lib_test.launch)
endif()
