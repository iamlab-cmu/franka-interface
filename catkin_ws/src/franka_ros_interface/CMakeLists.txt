cmake_minimum_required(VERSION 2.8.3)
project(franka_ros_interface)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
  rospy
  franka_interface_msgs
  tf
)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set(franka-interface-common_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../../../franka-interface-common/include")
set(franka-interface-common_lib "${CMAKE_CURRENT_SOURCE_DIR}/../../../build/franka-interface-common/")
set(proto_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../../../build/franka-interface/proto/")
set(proto "${CMAKE_CURRENT_SOURCE_DIR}/../../../build/franka-interface/proto/")

find_package(Boost COMPONENTS REQUIRED filesystem system thread)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${franka-interface-common_INCLUDE_DIRS})
include_directories(${proto_INCLUDE_DIRS})

LINK_DIRECTORIES(${franka-interface-common_lib})
LINK_DIRECTORIES(${proto})

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
   INCLUDE_DIRS include
#  LIBRARIES franka_ros_interface
   CATKIN_DEPENDS roscpp rospy actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/shared_memory_handler.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/franka_ros_interface_node.cpp)
add_executable(execute_skill_action_server_node src/execute_skill_action_server.cpp src/execute_skill_action_server_node.cpp)

add_executable(robot_state_publisher_node 
  src/robot_state_publisher.cpp 
  src/robot_state_publisher_node.cpp
)
 
add_executable(franka_interface_status_publisher_node src/franka_interface_status_publisher.cpp src/franka_interface_status_publisher_node.cpp)
add_executable(run_loop_process_info_state_publisher_node src/run_loop_process_info_state_publisher.cpp src/run_loop_process_info_state_publisher_node.cpp)
add_executable(get_current_robot_state_server_node src/get_current_robot_state_server.cpp src/get_current_robot_state_server_node.cpp)
add_executable(get_current_franka_interface_status_server_node src/get_current_franka_interface_status_server.cpp src/get_current_franka_interface_status_server_node.cpp)

# Sensor data 
add_executable(sensor_data_subscriber_node 
  src/sensor_data/sensor_subscriber.cpp
  src/sensor_data/sensor_subscriber_handler.cpp
)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies( execute_skill_action_server_node
${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies( get_current_robot_state_server_node
${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies( get_current_robot_state_server_node
${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies( sensor_data_subscriber_node
${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(
  execute_skill_action_server_node
  Threads::Threads
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  rt
  proto
  franka-interface-common
  ${PROJECT_NAME}
  ${Protobuf_LIBRARIES}
)
target_link_libraries(
  robot_state_publisher_node
  Threads::Threads
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  rt
  proto
  franka-interface-common
  ${PROJECT_NAME}
  ${Protobuf_LIBRARIES}
)
target_link_libraries(
  franka_interface_status_publisher_node
  Threads::Threads
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  rt
  proto
  franka-interface-common
  ${PROJECT_NAME}
  ${Protobuf_LIBRARIES}
)
target_link_libraries(
  run_loop_process_info_state_publisher_node
  Threads::Threads
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  rt
  proto
  franka-interface-common
  ${PROJECT_NAME}
  ${Protobuf_LIBRARIES}
)
target_link_libraries(
  get_current_robot_state_server_node
  Threads::Threads
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  rt
  proto
  ${PROJECT_NAME}
  ${Protobuf_LIBRARIES}
)
target_link_libraries(
  get_current_franka_interface_status_server_node
  Threads::Threads
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  rt
  proto
  ${PROJECT_NAME}
  ${Protobuf_LIBRARIES}
)
target_link_libraries(
  sensor_data_subscriber_node
  Threads::Threads
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  rt
  proto
  franka-interface-common
  ${PROJECT_NAME}
  ${Protobuf_LIBRARIES}
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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_franka_ros_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
