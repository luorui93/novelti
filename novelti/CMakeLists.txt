cmake_minimum_required(VERSION 2.8.11.2)
project(novelti)

#This is how you compile it with debug info
#cd build/novelti/novelti
#cmake -D CMAKE_BUILD_TYPE=Debug  -D CATKIN_DEVEL_PREFIX=../../../devel ../../../src/novelti/novelti
#make

#SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++0x -O3")  #optimized
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++0x -Winline -gdwarf-2 -g3  " ) #for debug
SET( CWAVE_LIB_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/lib") 

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  rosbag
  geometry_msgs
  message_generation
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


########### rviz plugin for IntMap and FloatMap displays ##############
## This plugin includes Qt widgets, so we must include Qt like so:
# find_package(Qt5 COMPONENTS Core Gui REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL)
# set variable names already used with Qt4
set(QT_LIBRARIES Qt5::Widgets)
set(QTVERSION ${Qt5Widgets_VERSION})
# include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt5_wrap_cpp(MOC_FILES
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
  OrientationDisplay.msg
  OrientationPdf.msg
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
   geometry_msgs
   novelti
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
  #LIBRARIES yaml-cpp
  CATKIN_DEPENDS 
    roscpp 
    message_runtime
    rosbag
    
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
# add_library(novelti
#   src/${PROJECT_NAME}/novelti.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(novelti ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
####add_executable(arc src/arc.cpp src/arc_list.cpp src/map.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(novelti_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})




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

#add_executable(chatter src/chatter.cpp)
#target_link_libraries(chatter
#    ${catkin_LIBRARIES}
#)


# # add_executable(map_divider src/node_map_divider.cpp src/map_divider.cpp)
# # target_compile_definitions(map_divider PRIVATE  ROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO  CWAVE2_FLOAT_STARS=1 CWAVE2_PROC=1 CWAVE2_TRACK=1 -DCWAVE2_FLOAT_MERGE)
# # target_link_libraries(map_divider
# #     ${catkin_LIBRARIES}
# #     ${CWAVE_LIB_DIR}/libcwaveproc.a
# # )
# # add_dependencies(map_divider ${PROJECT_NAME}_generate_messages_cpp ${CWAVE_LIB_DIR}/libcwaveproc.a)
# # 
# # add_executable(best_pose_finder src/node_best_pose_finder.cpp src/best_pose_finder.cpp src/best_pose_finder_quasi_opt.cpp)
# # target_compile_definitions(best_pose_finder PRIVATE ROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO CWAVE2_FLOAT_STARS=1 CWAVE2_PROC=1 CWAVE2_TRACK=1 -DCWAVE2_FLOAT_MERGE)
# # target_link_libraries(best_pose_finder
# #     ${catkin_LIBRARIES}
# #     ${CWAVE_LIB_DIR}/libcwaveproc.a
# # )
# # add_dependencies(best_pose_finder ${PROJECT_NAME}_generate_messages_cpp ${CWAVE_LIB_DIR}/libcwaveproc.a)
# # 
# # add_executable(inference_unit src/node_inference_unit.cpp src/inference_unit.cpp)
# # target_link_libraries(inference_unit
# #     ${catkin_LIBRARIES}
# # )
# # add_dependencies(inference_unit ${PROJECT_NAME}_generate_messages_cpp ${CWAVE_LIB_DIR}/libcwaveproc.a)

add_executable(novelti_shared_control 
    src/novelti_shared_control.cpp 
    src/position_control.cpp 
    src/map_divider.cpp 
    src/inference_matrix.cpp 
    src/best_pose_finder.cpp 
    src/best_pose_finder_quasi_opt.cpp 
#     src/inference_unit.cpp 
    src/orientation_control.cpp 
    src/opt_orientation_selector.cpp
    src/orientation_disk_divider.cpp
)
target_compile_definitions(novelti_shared_control 
    PRIVATE  
        ROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO  
        CWAVE2_FLOAT_STARS=1 
        CWAVE2_PROC=1 
        CWAVE2_TRACK=1 
        -DCWAVE2_FLOAT_MERGE)
target_link_libraries(novelti_shared_control
    ${catkin_LIBRARIES}
    ${CWAVE_LIB_DIR}/libcwaveproc.a
    ${CWAVE_LIB_DIR}/libcwavesector.a
    ${CWAVE_LIB_DIR}/libcwavesector-3.2.a
)
add_dependencies(novelti_shared_control ${PROJECT_NAME}_generate_messages_cpp ${CWAVE_LIB_DIR}/libcwaveproc.a ${CWAVE_LIB_DIR}/libcwavesector.a ${CWAVE_LIB_DIR}/libcwavesector-3.2.a)


add_executable(diff_drive_robot_model src/node_robot_model_diff_drive.cpp)
target_compile_definitions(diff_drive_robot_model PRIVATE CWAVE2_FLOAT_STARS=1 CWAVE2_PROC=1 CWAVE2_TRACK=1 -DCWAVE2_FLOAT_MERGE)
target_link_libraries(diff_drive_robot_model
   ${catkin_LIBRARIES}
   ${CWAVE_LIB_DIR}/libcwaveproc.a
)
add_dependencies(diff_drive_robot_model ${PROJECT_NAME}_generate_messages_cpp)


add_executable(human_model src/node_human_model.cpp)
target_link_libraries(human_model
   ${catkin_LIBRARIES}
)
add_dependencies(human_model ${PROJECT_NAME}_generate_messages_cpp)


add_library(novelti_rviz_plugin ${SOURCE_FILES})
target_link_libraries(novelti_rviz_plugin ${catkin_LIBRARIES} ${QT_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS} ${Boost_LIBRARIES})
add_dependencies(novelti_rviz_plugin ${PROJECT_NAME}_generate_messages_cpp)


######################### TOOLS ##################################
add_executable(bag_processor src/node_bag_processor.cpp)
target_compile_definitions(bag_processor PRIVATE  ROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO  CWAVE2_FLOAT_STARS=1 CWAVE2_PROC=1 CWAVE2_TRACK=1 -DCWAVE2_FLOAT_MERGE)    #add to remove loggin at compile time: ROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO
target_link_libraries(bag_processor
   ${catkin_LIBRARIES}
   ${CWAVE_LIB_DIR}/libcwaveproc.a
   -lyaml-cpp
)
add_dependencies(bag_processor ${PROJECT_NAME}_generate_messages_cpp)

add_executable(cwave_cmdline src/cwave_cmdline.cpp)
target_compile_definitions(cwave_cmdline PRIVATE CWAVE2_FLOAT_STARS=1 CWAVE2_PROC=1 CWAVE2_TRACK=1 -DCWAVE2_FLOAT_MERGE)
target_link_libraries(cwave_cmdline
    ${CWAVE_LIB_DIR}/libcwaveproc.a
#     libSDL2.so
#     libSDL2_ttf.so
    )
add_dependencies(cwave_cmdline 
    ${CWAVE_LIB_DIR}/libcwaveproc.a
)


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
# install(TARGETS novelti novelti_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_novelti.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
