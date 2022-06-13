cmake_minimum_required(VERSION 2.8.3)
project(karcher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf 
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES karcher
#  CATKIN_DEPENDS geometry_msgsmessage_generation roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/tinkerforge
  ${PROJECT_SOURCE_DIR}/include/WiringPi
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/karcher.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/karcher_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

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

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_karcher.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(brs_check_lidar src/brs_check_lidar.cpp)
target_link_libraries(brs_check_lidar ${catkin_LIBRARIES})

add_executable(brs_vis_both src/brs_vis_both.cpp
src/debug_header.cpp
)
target_link_libraries(brs_vis_both ${catkin_LIBRARIES})

add_executable(karcher_teleop_com_def src/karcher_teleop_com_def.cpp)
target_link_libraries(karcher_teleop_com_def ${catkin_LIBRARIES})

add_executable(karcher_teleop_termios src/karcher_teleop_termios.cpp)
target_link_libraries(karcher_teleop_termios ${catkin_LIBRARIES})

add_executable(karcher_teleop_old src/karcher_teleop_old.cpp)
target_link_libraries(karcher_teleop_old ${catkin_LIBRARIES})

add_executable(Speed_control src/Speed_control.cpp)
target_link_libraries(Speed_control ${catkin_LIBRARIES})

add_executable(Heading_control src/Heading_control.cpp)
target_link_libraries(Heading_control ${catkin_LIBRARIES})

add_executable(Turn_control src/Turn_control.cpp)
target_link_libraries(Turn_control ${catkin_LIBRARIES})

add_executable(Distance_control src/Distance_control.cpp)
target_link_libraries(Distance_control ${catkin_LIBRARIES})

add_executable(Navigation_ZZ src/Navigation_ZZ.cpp)
target_link_libraries(Navigation_ZZ ${catkin_LIBRARIES})

add_executable(command_office src/command_office.cpp)
target_link_libraries(command_office ${catkin_LIBRARIES})

add_executable(brs_sense src/brs_sense.cpp
src/debug_header.cpp
)
target_link_libraries(brs_sense ${catkin_LIBRARIES})

add_executable(brs_sense_pwm src/brs_sense_pwm.cpp
src/debug_header.cpp
)
target_link_libraries(brs_sense_pwm ${catkin_LIBRARIES})

add_executable(brs_pose_data src/brs_pose_data.cpp)
target_link_libraries(brs_pose_data ${catkin_LIBRARIES})

add_executable(brs_left_wall src/brs_left_wall.cpp
src/debug_header.cpp
src/tuning_header.cpp
)
target_link_libraries(brs_left_wall ${catkin_LIBRARIES})

add_executable(brs_right_wall src/brs_right_wall.cpp
src/debug_header.cpp
src/tuning_header.cpp
)
target_link_libraries(brs_right_wall ${catkin_LIBRARIES})

add_executable(brs_left_wall_pwm src/brs_left_wall_pwm.cpp
src/debug_header.cpp
src/tuning_header.cpp
)
target_link_libraries(brs_left_wall_pwm ${catkin_LIBRARIES})

add_executable(brs_right_wall_pwm src/brs_right_wall_pwm.cpp
src/debug_header.cpp
src/tuning_header.cpp
)
target_link_libraries(brs_right_wall_pwm ${catkin_LIBRARIES})

add_executable(brs_left_step src/brs_left_step.cpp
src/debug_header.cpp
src/tuning_header.cpp
)
target_link_libraries(brs_left_step ${catkin_LIBRARIES})

add_executable(brs_right_step src/brs_right_step.cpp
src/debug_header.cpp
src/tuning_header.cpp
)
target_link_libraries(brs_right_step ${catkin_LIBRARIES})

add_executable(brs_dt src/brs_dt.cpp)
target_link_libraries(brs_dt ${catkin_LIBRARIES})

add_executable(bthere src/bthere.cpp)
target_link_libraries(bthere ${catkin_LIBRARIES})

add_executable(brs_debug_status src/brs_debug_status.cpp)
target_link_libraries(brs_debug_status ${catkin_LIBRARIES})

add_executable(Position_control src/Position_control.cpp)
target_link_libraries(Position_control ${catkin_LIBRARIES})

add_executable(brs_str src/brs_str.cpp)
target_link_libraries(brs_str ${catkin_LIBRARIES})

add_executable(brs_checking src/brs_checking.cpp)
target_link_libraries(brs_checking ${catkin_LIBRARIES})

add_executable(brs_odom src/brs_odom.cpp)
target_link_libraries(brs_odom ${catkin_LIBRARIES})

add_executable(brs_pose_without_ekf src/brs_pose_without_ekf.cpp)
target_link_libraries(brs_pose_without_ekf ${catkin_LIBRARIES})

add_executable(brs_teleop src/brs_teleop.cpp)
target_link_libraries(brs_teleop ${catkin_LIBRARIES})

add_executable(brs_teleop_wss src/brs_teleop_wss.cpp)
target_link_libraries(brs_teleop_wss ${catkin_LIBRARIES})

add_executable(teleop_pwm src/teleop_pwm.cpp)
target_link_libraries(teleop_pwm ${catkin_LIBRARIES})

add_executable(brs_pose_str src/brs_pose_str.cpp
src/debug_header.cpp
src/tuning_header.cpp
)
target_link_libraries(brs_pose_str ${catkin_LIBRARIES})

add_executable(brs_node_check src/brs_node_check.cpp
src/debug_header.cpp
)
target_link_libraries(brs_node_check ${catkin_LIBRARIES})

add_executable(brs_current src/brs_current.cpp)
target_link_libraries(brs_current ${catkin_LIBRARIES})
