cmake_minimum_required(VERSION 2.8.3)
project(robotiq_mm_ros)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  std_msgs
  actionlib
  actionlib_msgs
  message_generation 
)

catkin_python_setup()

add_service_files(
  FILES 
  GetGripperPos.srv 
)

add_action_files(
  DIRECTORY action
  FILES
  GripperCommand.action
)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(
  LIBRARIES robotiq_mm_ros
  CATKIN_DEPENDS 
  actionlib 
  actionlib_msgs 
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/robotiq_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)