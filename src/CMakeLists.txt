cmake_minimum_required(VERSION 2.8.3)
project(uwb_loc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  mavros_extras
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_example_node serial_example_node.cpp)

target_link_libraries(serial_example_node
  ${catkin_LIBRARIES}
)

