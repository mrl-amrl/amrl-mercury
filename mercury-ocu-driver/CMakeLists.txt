cmake_minimum_required(VERSION 2.8.3)
project(mercury_ocu_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  Joy.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
