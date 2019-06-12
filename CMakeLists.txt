cmake_minimum_required(VERSION 2.8.3)
project(pokeman)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  geometry_msgs
#  apriltags
        )

find_package(
  OpenCV REQUIRED
)

catkin_package()

include_directories(
   ${catkin_INCLUDE_DIRS}
   include
)

add_executable(pokeman_search src/pokemon_searching.cpp)
#add_dependencies(pokeman_search apriltags)

target_link_libraries(
  pokeman_search
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
