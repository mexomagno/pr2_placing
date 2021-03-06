cmake_minimum_required(VERSION 2.8.3)
project(memoria)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    pcl_ros
    roscpp
    sensor_msgs
    actionlib
    pr2_controllers_msgs
    message_generation
    moveit_ros_planning_interface
    gazebo_msgs
    # filters
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LookAtMsg.msg
  ErrorMsg.msg
#  BaseDriverMsg.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  LookAt.srv
  SearchSurface.srv
  GoToPose.srv
  BaseDriver.srv
  GripperDriver.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES memoria
 CATKIN_DEPENDS message_runtime 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(memoria
#   src/${PROJECT_NAME}/memoria.cpp
# )

## Declare a cpp executable
# add_executable(memoria_node src/memoria_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(memoria_node memoria_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(memoria_node
#   ${catkin_LIBRARIES}
# )

#########################

#### EJECUTABLE SERVICIO LOOKAT

add_executable(LookAt src/LookAt.cpp)
add_dependencies(LookAt memoria_generate_messages_cpp)
target_link_libraries(LookAt ${catkin_LIBRARIES})
#add_dependencies(LookAt memoria_gencpp)

#### EJECUTABLE SERVICIO SEARCHSURFACE

add_executable(SearchSurface src/SearchSurface.cpp)
add_dependencies(SearchSurface memoria_generate_messages_cpp)
target_link_libraries(SearchSurface ${catkin_LIBRARIES})

#### EJECUTABLE BUSCAR SUPERFICIE

add_executable(buscar_superficie src/buscar_superficie.cpp)
add_dependencies(buscar_superficie memoria_generate_messages_cpp)
target_link_libraries(buscar_superficie ${catkin_LIBRARIES})
#add_dependencies(buscar_superficie memoria_gencpp)

#### EJECUTABLE GOTOPOSE

add_executable(GoToPose src/GoToPose.cpp)
add_dependencies(GoToPose memoria_generate_messages_cpp)
target_link_libraries(GoToPose ${catkin_LIBRARIES})

#### EJECUTABLE BASEDRIVER

add_executable(BaseDriver src/BaseDriver.cpp)
add_dependencies(BaseDriver memoria_generate_messages_cpp)
target_link_libraries(BaseDriver ${catkin_LIBRARIES})

#### EJECUTABLE GRIPPERDRIVER

add_executable(GripperDriver src/GripperDriver.cpp)
add_dependencies(GripperDriver memoria_generate_messages_cpp)
target_link_libraries(GripperDriver ${catkin_LIBRARIES})

#### EJECUTABLE PLACING

add_executable(placing src/placing.cpp)
add_dependencies(placing memoria_generate_messages_cpp)
target_link_libraries(placing ${catkin_LIBRARIES})


# ***** Ejecutables de test
#### EJECUTABLE TEST BASEDRIVER

add_executable(basedrivertest src/basedrivertest.cpp)
add_dependencies(basedrivertest memoria_generate_messages_cpp)
target_link_libraries(basedrivertest ${catkin_LIBRARIES})

#### EJECUTABLE TEST GOTOPOSE

add_executable(gotoposetest src/gotoposetest.cpp)
add_dependencies(gotoposetest memoria_generate_messages_cpp)
target_link_libraries(gotoposetest ${catkin_LIBRARIES})

#### EJECUTABLE TEST GRIPPERDRIVER

add_executable(gripperdrivertest src/gripperdrivertest.cpp)
add_dependencies(gripperdrivertest memoria_generate_messages_cpp)
target_link_libraries(gripperdrivertest ${catkin_LIBRARIES})

#### EJECUTABLE TEST GRASPING

add_executable(graspingtest src/graspingtest.cpp)
add_dependencies(graspingtest memoria_generate_messages_cpp)
target_link_libraries(graspingtest ${catkin_LIBRARIES})

#### EJECUTABLE GRIPPER SCANNER

add_executable(gripper_scanner src/gripper_scanner.cpp)
add_dependencies(gripper_scanner memoria_generate_messages_cpp)
target_link_libraries(gripper_scanner ${catkin_LIBRARIES})

#### EJECUTABLE PICKPLACE

add_executable(pickplace src/pickplace.cpp)
target_link_libraries(pickplace ${catkin_LIBRARIES})

# Se trasladó a memoria2
# add_executable(grippernode src/grippernode.cpp)
# target_link_libraries(grippernode ${catkin_LIBRARIES})

#### TEST SELF FILTER

# #set(SF_D "/opt/ros/hydro/share/pr2_navigation_self_filter")
# add_executable(self_filter src/self_filter.cpp)
# add_dependencies(self_filter filters)
# target_link_libraries(self_filter ${catkin_LIBRARIES} "/opt/ros/hydro/lib/libresource_retriever.so")

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
# install(TARGETS memoria memoria_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_memoria.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
