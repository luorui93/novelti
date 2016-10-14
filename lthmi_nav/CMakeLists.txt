cmake_minimum_required(VERSION 2.8.3)
project(lthmi_nav)

#This is how you compile it with debug info
#cd build/lthmi_nav/lthmi_nav
#cmake -D CMAKE_BUILD_TYPE=Debug  -D CATKIN_DEVEL_PREFIX=../../../devel ../../../src/lthmi_nav/lthmi_nav
#make

#SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++0x -O3")  #optimized
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++0x -Winline -gdwarf-2 -g3  " ) #for debug
SET( CWAVE_LIB_DIR  "/home/sd/ws_old/src/cwave") 
SET( CWAVE_INCLUDE_DIR  "/home/sd/ws_old/src/cwave/include")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


########### rviz plugin for IntMap and FloatMap displays ##############
## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  include/${PROJECT_NAME}/rviz_int_map_display.h
  include/${PROJECT_NAME}/rviz_float_map_display.h
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/rviz_int_map_display.cpp 
  src/rviz_float_map_display.cpp
  ${MOC_FILES}
)






## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
 catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
  Command.msg
  FloatMap.msg
  IntMap.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  StartExperiment.srv
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
   nav_msgs
   lthmi_nav
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  INCLUDE_DIRS 
    include
    /usr/include/OGRE  #temporary dirty workaround
    ${OGRE_OV_INCLUDE_DIRS}
#  LIBRARIES lthmi_nav
  CATKIN_DEPENDS roscpp message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  /usr/include/OGRE  #temporary dirty workaround
  ${OGRE_OV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CWAVE_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(lthmi_nav
#   src/${PROJECT_NAME}/lthmi_nav.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(lthmi_nav ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
####add_executable(arc src/arc.cpp src/arc_list.cpp src/map.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(lthmi_nav_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})




# add_executable(test_best_pose_finder test/test_best_pose_finder.cpp src/best_pose_finder.cpp src/map.cpp)
# target_link_libraries(test_best_pose_finder
#    ${catkin_LIBRARIES}
#    ncurses
# )
# add_dependencies(test_best_pose_finder ${PROJECT_NAME}_generate_messages_cpp)
# 
# add_executable(best_pose_finder_node src/best_pose_finder_node.cpp src/map.cpp)
# target_link_libraries(best_pose_finder_node
#    ${catkin_LIBRARIES}
#    ncurses
# )
# add_dependencies(best_pose_finder_node ${PROJECT_NAME}_generate_messages_cpp)
# 
# 
# add_executable(space_divider_node src/space_divider_node.cpp src/map.cpp)
# target_link_libraries(space_divider_node
#    ${catkin_LIBRARIES}
#    ncurses
# )
# add_dependencies(space_divider_node ${PROJECT_NAME}_generate_messages_cpp)


add_executable(map_divider src/node_map_divider.cpp src/map_divider.cpp)
target_compile_definitions(map_divider PRIVATE CWAVE2_FLOAT_STARS=1 CWAVE2_PROC=1 CWAVE2_TRACK=1 -DCWAVE2_FLOAT_MERGE)
target_link_libraries(map_divider
    ${catkin_LIBRARIES}
    ${CWAVE_LIB_DIR}/libcwaveproc.a
)
add_dependencies(map_divider ${PROJECT_NAME}_generate_messages_cpp ${CWAVE_LIB_DIR}/libcwaveproc.a)

add_executable(best_pose_finder src/node_best_pose_finder.cpp src/best_pose_finder.cpp)
target_compile_definitions(best_pose_finder PRIVATE CWAVE2_FLOAT_STARS=1 CWAVE2_PROC=1 CWAVE2_TRACK=1 -DCWAVE2_FLOAT_MERGE)
target_link_libraries(best_pose_finder
    ${catkin_LIBRARIES}
    ${CWAVE_LIB_DIR}/libcwaveproc.a
)
add_dependencies(best_pose_finder ${PROJECT_NAME}_generate_messages_cpp ${CWAVE_LIB_DIR}/libcwaveproc.a)


add_library(lthmi_nav_rviz_plugin ${SOURCE_FILES})
target_link_libraries(lthmi_nav_rviz_plugin ${catkin_LIBRARIES} ${QT_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS} ${Boost_LIBRARIES})


# add_executable(map_publisher src/map_publisher.cpp)
# target_link_libraries(map_publisher
#    ${catkin_LIBRARIES}
#    ncurses
# )
# add_dependencies(map_publisher ${PROJECT_NAME}_generate_messages_cpp)
# 
# add_executable(robot_model src/robot_model.cpp)
# target_link_libraries(robot_model
#    ${catkin_LIBRARIES}
# )
# add_dependencies(robot_model ${PROJECT_NAME}_generate_messages_cpp)

#install(TARGETS default_plugin
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

  
  
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
# install(TARGETS lthmi_nav lthmi_nav_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lthmi_nav.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
