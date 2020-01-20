cmake_minimum_required(VERSION 2.8.3)
project(mercury_joy)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  rospy
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  Joy.msg
)

add_service_files(
  FILES
  Rumble.srv
)

generate_dynamic_reconfigure_options(
  cfg/Joy.cfg
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
