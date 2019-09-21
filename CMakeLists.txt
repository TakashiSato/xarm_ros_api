cmake_minimum_required(VERSION 2.8.3)
project(xarm_ros_api)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  xarm_planner
  xarm_msgs
)

catkin_python_setup()

catkin_package(
 LIBRARIES xarm_ros_api
 CATKIN_DEPENDS
    rospy
    xarm_planner
    xarm_msgs
)
