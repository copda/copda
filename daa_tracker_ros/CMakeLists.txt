cmake_minimum_required(VERSION 3.5.1)
project(daa_tracker_ros)

set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  daa_color_classification
  image_geometry
  image_transport
  message_generation
  roscpp
  roslaunch
  sensor_msgs
  std_msgs
  tf
  vision_msgs
  visualization_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED COMPONENTS core photo imgcodecs highgui)
find_package(Eigen3 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
add_message_files(
  FILES
  Track.msg
  TrackArray.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS
    cv_bridge
    daa_color_classification
    image_geometry
    message_runtime
    roscpp
    sensor_msgs
    std_msgs
    tf
    vision_msgs
    visualization_msgs
  DEPENDS
    EIGEN3
    OpenCV
)

###########
## Build ##
###########

add_subdirectory(src/daa_tracker)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(tracker_node src/tracker_node.cpp)
add_dependencies(tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracker_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  daa_tracker
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
#catkin_install_python(PROGRAMS
#  scripts/daa_tracker_ros.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# Mark executables for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS
  tracker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark libraries for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS
  cliquer
  daa_tracker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

roslaunch_add_file_check(launch)
