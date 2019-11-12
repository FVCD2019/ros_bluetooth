cmake_minimum_required(VERSION 2.8.3)
project(ros_bluetooth)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  move_base_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mobile_robot_pkg
#  CATKIN_DEPENDS move_base_msgs nav_msgs robot_pose_ekf robot_state_publisher roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

