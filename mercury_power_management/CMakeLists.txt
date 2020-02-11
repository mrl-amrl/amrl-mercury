cmake_minimum_required(VERSION 2.8.3)
project(mercury_power_management)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  FILES
  States.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
 CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

