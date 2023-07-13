cmake_minimum_required(VERSION 3.10)

project(digit_panda)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_generation)

catkin_python_setup()


add_service_files(
  FILES
  Command.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS rospy)

install(PROGRAMS
     nodes/digit_node
     nodes/visualizer_node
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)