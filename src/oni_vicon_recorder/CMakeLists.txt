cmake_minimum_required(VERSION 2.8.3)
project(oni_vicon_recorder)

find_package(catkin REQUIRED
    roscpp
    std_msgs
    message_generation
    actionlib_msgs
    actionlib
    rgbd_sensor
    vicon_sdk
    depth_sensor_vicon_calibration)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
   DIRECTORY srv
   FILES
   ViconObjects.srv
   ViconObjectPose.srv
   VerifyObjectExists.srv
)

add_action_files(
  DIRECTORY action
  FILES
  Record.action
  RunDepthSensor.action
  ChangeDepthSensorMode.action
  ConnectToVicon.action
)

generate_messages(
  DEPENDENCIES
  std_msgs actionlib_msgs std_msgs geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES oni_vicon_recorder
 CATKIN_DEPENDS actionlib_msgs actionlib vicon_sdk rgbd_sensor depth_sensor_vicon_calibration
 DEPENDS OpenNI
)

###########
## Build ##
###########
set(oni_vicon_recorder_SOURCES
  src/kinect.cc
  src/oni_recorder.cpp
  src/vicon_recorder.cpp
  src/oni_vicon_recorder.cpp
  src/frame_time_tracker.cpp
)

set(oni_vicon_recorder_HEADERS
  include/${PROJECT_NAME}/kinect.h
  include/${PROJECT_NAME}/oni_recorder.hpp
  include/${PROJECT_NAME}/vicon_recorder.hpp
  include/${PROJECT_NAME}/oni_vicon_recorder.hpp
  include/${PROJECT_NAME}/frame_time_tracker.hpp
  include/${PROJECT_NAME}/namespaces.hpp
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    "/usr/include/ni/" # FIXME superbly uggly
)

add_library(oni_vicon_recorder ${oni_vicon_recorder_SOURCES} ${oni_vicon_recorder_HEADERS})
target_link_libraries(oni_vicon_recorder
    ${catkin_LIBRARIES}
    "ViconDataStreamSDK_CPP"
    OpenNI
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    yaml-cpp
)

add_executable(oni_vicon_recorder_node src/oni_vicon_recorder_node.cpp)
target_link_libraries(oni_vicon_recorder_node
    oni_vicon_recorder
    ${catkin_LIBRARIES}
    OpenNI
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS oni_vicon_recorder oni_vicon_recorder_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_oni_vicon_recorder.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
