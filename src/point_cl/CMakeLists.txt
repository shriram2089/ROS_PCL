cmake_minimum_required(VERSION 3.0.2)
project(point_cl)

find_package(catkin REQUIRED COMPONENTS
  
  rospy
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY code
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
