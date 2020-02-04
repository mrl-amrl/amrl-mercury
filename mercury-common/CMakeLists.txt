cmake_minimum_required(VERSION 2.8.3)
project(mercury_common)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_service_files(
  FILES
  SetEnabled.srv
)
add_message_files(
  DIRECTORY
    msg
  FILES
    ManipulatorStatus.msg
    MovementFeedback.msg
    EposError.msg
    RobotsFeedback.msg
    Ping.msg
    PowerFeedback.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS message_generation message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

