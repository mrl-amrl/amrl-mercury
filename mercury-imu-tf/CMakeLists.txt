cmake_minimum_required(VERSION 2.8.3)
project(mercury_imu_tf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
)

add_executable(imu_tf src/imu-tf.cpp)

target_link_libraries(imu_tf
  ${catkin_LIBRARIES}
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
