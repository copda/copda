cmake_minimum_required(VERSION 3.5.1)
project(daa_knowledge_base_ros)

find_package(catkin REQUIRED COMPONENTS
  grpc
  message_generation
  roscpp
  roslaunch
  std_msgs
)

catkin_python_setup()

add_service_files(
  FILES
  Query.srv
  Update.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  roscpp
  std_msgs
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  nodes/knowledge_base_server.py
  nodes/knowledge_base_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

roslaunch_add_file_check(launch)
