cmake_minimum_required(VERSION 2.8.3)
project(path_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
)








catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_publisher src/path_publisher.cpp)
target_link_libraries(path_publisher ${catkin_LIBRARIES})
