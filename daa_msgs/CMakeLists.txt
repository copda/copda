cmake_minimum_required(VERSION 3.5.1)
project(daa_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  vision_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  AnchoredObject.msg
  AnchoredObjectArray.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    vision_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    vision_msgs
)
