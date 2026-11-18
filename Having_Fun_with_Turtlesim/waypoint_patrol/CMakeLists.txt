cmake_minimum_required(VERSION 3.0.2)
project(waypoint_patrol)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs geometry_msgs turtlesim message_generation)

add_message_files(FILES MissionGoal.msg MissionStatus.msg)
add_service_files(FILES StartMission.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS rospy std_msgs geometry_msgs turtlesim message_runtime)

catkin_install_python(PROGRAMS
  scripts/waypoint_controller.py
  scripts/mission_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
