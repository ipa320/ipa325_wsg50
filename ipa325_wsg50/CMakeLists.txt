cmake_minimum_required(VERSION 3.1.0)
project(ipa325_wsg50)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin
    REQUIRED COMPONENTS
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
    sensor_msgs
    trajectory_msgs
    message_generation
    std_srvs
##  additional packages
)

## System dependencies are found with CMake's conventions
find_package(Boost
    REQUIRED COMPONENTS
    system
)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# set(CMAKE_BUILD_TYPE Debug)


#######################################
## Declare ROS messages and services ##
#######################################

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY
    action
  FILES
    LwrRunControlScheme.action
    WSG50Homing.action
    WSG50PrePositionFingers.action
    WSG50GraspPart.action
    WSG50ReleasePart.action
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    systState.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
#    WSG50Disconnect.srv
    setAcceleration.srv
    setForceLimit.srv
    setSoftLimits.srv
    tareForceSensor.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  trajectory_msgs
  std_msgs
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
 LIBRARIES wsg50_ros
  CATKIN_DEPENDS
    roscpp
    actionlib
    actionlib_msgs
    trajectory_msgs
    std_msgs
    message_runtime
#  DEPENDS system_lib
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS} #include/3rdParty/fri
)

## Specify libraries to link a library or executable target against
#link_directories(lib
#  lib/3rdParty/fri
#)

## Declare a cpp library
add_library(wsg50_ros
  src/WSG50Subject.cpp
    src/WSG50Controller.cpp
    src/WSG50Communicator.cpp
    src/WSG50ROSNode.cpp
    src/WSG50RosObserver.cpp
    src/WSG50RosSubject.cpp
)
add_dependencies(wsg50_ros ${PROJECT_NAME}_gencpp)
target_link_libraries(wsg50_ros
  ${catkin_LIBRARIES}
  pthread
  rt ## clock_gettime
)

## Declare a cpp executable
add_executable(schunk_testclient src/lwr_testclient.cpp)
add_dependencies(schunk_testclient ${PROJECT_NAME}_gencpp)

add_executable(schunk_gripper
    #src/testerGripper.cpp
    src/WSG50ROSNode.cpp
    src/WSG50Subject.cpp
    src/WSG50Controller.cpp
    src/WSG50Communicator.cpp
    src/WSG50ROSNode.cpp
    src/WSG50RosObserver.cpp
    src/WSG50RosSubject.cpp
)
add_dependencies(schunk_gripper ${PROJECT_NAME}_gencpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(kuka_lwr_node kuka_lwr_generate_messages_cpp)

target_link_libraries(schunk_testclient
  ${catkin_LIBRARIES}
  pthread
  rt ## clock_gettime
)

target_link_libraries(schunk_gripper
  ${catkin_LIBRARIES}
  pthread
  rt ## clock_gettime
)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS schunk_gripper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  etc
  launch
  meshes
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kuka_lwr.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
