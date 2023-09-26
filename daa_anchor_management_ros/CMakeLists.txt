cmake_minimum_required(VERSION 3.5.1)
project(daa_anchor_management_ros)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  nodes/anchor_management_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

roslaunch_add_file_check(launch)
