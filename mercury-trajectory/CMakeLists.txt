cmake_minimum_required(VERSION 2.8.3)
project(mercury_trajectory)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  rospy
  sensor_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Trajectory.cfg
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
