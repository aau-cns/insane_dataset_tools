cmake_minimum_required(VERSION 3.0.2)

project(insane_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   MotorSpeed.msg
   TagDistance.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

