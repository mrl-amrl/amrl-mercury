cmake_minimum_required(VERSION 2.8.3)
project(mercury_common)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_service_files(
  FILES
  SetEnabled.srv
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

