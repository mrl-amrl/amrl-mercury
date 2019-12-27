cmake_minimum_required(VERSION 2.8.3)
project(mercury_dynparam)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  message_generation
)

add_service_files(
  FILES
  Load.srv
  Default.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
