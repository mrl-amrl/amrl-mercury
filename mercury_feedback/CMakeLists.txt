cmake_minimum_required(VERSION 2.8.3)
project(mercury_feedback)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  rospy
  sensor_msgs
)



catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
