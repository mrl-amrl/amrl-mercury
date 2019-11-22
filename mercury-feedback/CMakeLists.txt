cmake_minimum_required(VERSION 2.8.3)
project(mercury_feedback)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  rospy
  sensor_msgs
  message_generation
)

add_service_files(
  FILES
    SetEnabled.srv
)

add_message_files(
  FILES
    EposError.msg
    ManipulatorStatus.msg
    MovementFeedback.msg
    RobotsFeedback.msg
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
