cmake_minimum_required(VERSION 3.0.2)
project(oryksbot_gazebo)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  pcl_conversions
  pcl_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bot_control
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
${catkin_INCLUDE_DIRS}
)

add_executable(odom_pub3 src/odom3.cpp)

target_link_libraries(odom_pub3
   ${catkin_LIBRARIES}
)
