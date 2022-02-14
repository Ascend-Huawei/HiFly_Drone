cmake_minimum_required(VERSION 3.0)
project(hifly_ros_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    sensor_msgs
    roscpp 
    rospy
)

add_message_files(
  FILES 
  Float32Array.msg
  Image.msg
  FaceDetection.msg
  AdaBins.msg
  HandDetection.msg
  ProcessVar.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime rospy std_msgs sensor_msgs
)


