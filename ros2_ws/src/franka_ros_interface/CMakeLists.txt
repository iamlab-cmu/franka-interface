cmake_minimum_required(VERSION 3.5)
project(franka_ros_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(franka_interface_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
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

link_directories(${franka-interface-common_lib})
link_directories(${proto})

include_directories(
  include
)


add_library(shared_memory_handler SHARED
  src/shared_memory_handler.cpp
)

target_include_directories(shared_memory_handler PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(shared_memory_handler 
  Threads::Threads
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  rt
  proto
  franka-interface-common
  ${Protobuf_LIBRARIES}
)

ament_target_dependencies(shared_memory_handler
    rclcpp
    rclcpp_action
    rclcpp_components
    franka_interface_msgs
)

add_executable(robot_state_publisher src/robot_state_publisher.cpp)
ament_target_dependencies(robot_state_publisher
    franka_interface_msgs
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
)

target_include_directories(robot_state_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(robot_state_publisher shared_memory_handler)

add_executable(run_loop_process_info_state_publisher src/run_loop_process_info_state_publisher.cpp)
ament_target_dependencies(run_loop_process_info_state_publisher
    franka_interface_msgs
    rclcpp
)

target_include_directories(run_loop_process_info_state_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(run_loop_process_info_state_publisher shared_memory_handler)

add_executable(franka_interface_status_publisher src/franka_interface_status_publisher.cpp)
ament_target_dependencies(franka_interface_status_publisher
    franka_interface_msgs
    rclcpp
)

target_include_directories(franka_interface_status_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(franka_interface_status_publisher shared_memory_handler)

add_executable(get_current_franka_interface_status_server src/get_current_franka_interface_status_server.cpp)
ament_target_dependencies(get_current_franka_interface_status_server
    franka_interface_msgs
    rclcpp
)

target_include_directories(get_current_franka_interface_status_server
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(get_current_franka_interface_status_server shared_memory_handler)

add_executable(get_current_robot_state_server src/get_current_robot_state_server.cpp)
ament_target_dependencies(get_current_robot_state_server
    franka_interface_msgs
    rclcpp
)

target_include_directories(get_current_robot_state_server
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(get_current_robot_state_server shared_memory_handler)

add_library(execute_skill_action_server SHARED
  src/execute_skill_action_server.cpp)
target_include_directories(execute_skill_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(execute_skill_action_server
  PRIVATE "FRANKA_ROS_INTERFACE_BUILDING_DLL")
ament_target_dependencies(execute_skill_action_server
  franka_interface_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
)

target_link_libraries(execute_skill_action_server shared_memory_handler)
rclcpp_components_register_node(execute_skill_action_server PLUGIN "franka_ros_interface::ExecuteSkillActionServer" EXECUTABLE franka_ros_interface_execute_skill_action_server)
install(TARGETS
  execute_skill_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_executable(sensor_data_subscriber src/sensor_data_subscriber.cpp)
ament_target_dependencies(sensor_data_subscriber
    franka_interface_msgs
    rclcpp
)

target_include_directories(sensor_data_subscriber
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(sensor_data_subscriber shared_memory_handler)

install(TARGETS
    robot_state_publisher
    run_loop_process_info_state_publisher
    franka_interface_status_publisher
    get_current_franka_interface_status_server
    get_current_robot_state_server
    sensor_data_subscriber
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
