cmake_minimum_required(VERSION 3.0.2)
project(daa_tools)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
 nodes/anchored_objects_to_markers.py
 nodes/detection3d_to_img.py
 nodes/detection3d_to_markers.py
 scripts/bag_to_imgs.py
 scripts/imgs_to_bag.py
 scripts/video_to_bag.py
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

roslaunch_add_file_check(launch)
