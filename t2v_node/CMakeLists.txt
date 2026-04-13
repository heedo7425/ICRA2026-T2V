cmake_minimum_required(VERSION 3.0.2)
project(t2v_node)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  T2VCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs rospy
)

catkin_install_python(PROGRAMS
  scripts/t2v_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
