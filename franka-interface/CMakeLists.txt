cmake_minimum_required (VERSION 3.4)
project (franka-interface
  VERSION 0.0.1 
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra)

## Dependencies
# find_package(Poco REQUIRED COMPONENTS Net Foundation)
find_package(Eigen3 REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Boost COMPONENTS REQUIRED filesystem system thread)
find_package(Protobuf REQUIRED)

add_subdirectory(proto)

include_directories( ${Boost_INCLUDE_DIRS} )

# find_library(franka-interface-common PATHS ../)
## Library

add_library(franka-interface SHARED
  src/feedback_controller/cartesian_impedance_feedback_controller.cpp
  src/feedback_controller/ee_cartesian_impedance_feedback_controller.cpp
  src/feedback_controller/feedback_controller.cpp
  src/feedback_controller/force_axis_impedence_feedback_controller.cpp
  src/feedback_controller/force_position_feedback_controller.cpp
  src/feedback_controller/joint_impedance_feedback_controller.cpp
  src/feedback_controller/noop_feedback_controller.cpp
  src/feedback_controller/pass_through_feedback_controller.cpp
  src/feedback_controller/set_internal_impedance_feedback_controller.cpp
  src/skills/base_meta_skill.cpp
  src/skills/base_skill.cpp
  src/skills/cartesian_pose_skill.cpp
  src/skills/force_torque_skill.cpp
  src/skills/gripper_skill.cpp
  src/skills/impedance_control_skill.cpp
  src/skills/joint_position_continuous_skill.cpp
  src/skills/joint_position_skill.cpp
  src/termination_handler/contact_termination_handler.cpp
  src/termination_handler/final_joint_termination_handler.cpp
  src/termination_handler/final_pose_termination_handler.cpp
  src/termination_handler/noop_termination_handler.cpp
  src/termination_handler/termination_handler.cpp
  src/termination_handler/time_termination_handler.cpp
  src/trajectory_generator/cubic_hermite_spline_joint_trajectory_generator.cpp
  src/trajectory_generator/cubic_hermite_spline_pose_trajectory_generator.cpp
  src/trajectory_generator/goal_pose_dmp_trajectory_generator.cpp
  src/trajectory_generator/gripper_trajectory_generator.cpp
  src/trajectory_generator/impulse_trajectory_generator.cpp
  src/trajectory_generator/joint_dmp_trajectory_generator.cpp
  src/trajectory_generator/joint_trajectory_generator.cpp
  src/trajectory_generator/linear_force_position_trajectory_generator.cpp
  src/trajectory_generator/linear_joint_trajectory_generator.cpp
  src/trajectory_generator/linear_pose_trajectory_generator.cpp
  src/trajectory_generator/min_jerk_joint_trajectory_generator.cpp
  src/trajectory_generator/min_jerk_pose_trajectory_generator.cpp
  src/trajectory_generator/pass_through_force_position_trajectory_generator.cpp
  src/trajectory_generator/pass_through_joint_trajectory_generator.cpp
  src/trajectory_generator/pass_through_pose_trajectory_generator.cpp
  src/trajectory_generator/pose_dmp_trajectory_generator.cpp
  src/trajectory_generator/pose_trajectory_generator.cpp
  src/trajectory_generator/relative_linear_pose_trajectory_generator.cpp
  src/trajectory_generator/relative_min_jerk_pose_trajectory_generator.cpp
  src/trajectory_generator/relative_pose_trajectory_generator.cpp
  src/trajectory_generator/sine_joint_trajectory_generator.cpp
  src/trajectory_generator/sine_pose_trajectory_generator.cpp
  src/trajectory_generator/stay_in_initial_joints_trajectory_generator.cpp
  src/trajectory_generator/stay_in_initial_pose_trajectory_generator.cpp
  src/duration.cpp
  src/feedback_controller_factory.cpp
  src/file_stream_logger.cpp
  src/robot_state_data.cpp
  src/run_loop.cpp
  src/run_loop_logger.cpp
  src/run_loop_shared_memory_handler.cpp
  src/save_robot_state_data_to_shared_memory_buffer.cpp
  src/sensor_data.cpp
  src/skill_info_manager.cpp
  src/termination_handler_factory.cpp
  src/trajectory_generator_factory.cpp
  src/utils/logger_utils.cpp
  src/sensor_data_manager.cpp
)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../franka-interface-common")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../libfranka")
include_directories("${CMAKE_CURRENT_BINARY_DIR}")
include_directories(${PROTOBUF_INCLUDE_DIR})
include_directories(include)


# Add namespace
# add_library(FrankaInterface::FrankaInterface ALIAS franka-interface)

target_link_libraries(franka-interface PUBLIC
  Eigen3::Eigen3
  Threads::Threads
  rt
  proto
  franka-interface-common
  Franka::Franka
  ${Boost_LIBRARIES}
  ${Protobuf_LIBRARIES}
)


target_include_directories(franka-interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set_target_properties(franka-interface PROPERTIES PUBLIC_HEADER include/run_loop.h)

target_include_directories(franka-interface PRIVATE src)
# target_include_directories(franka-interface PUBLIC proto)
target_include_directories(franka-interface PRIVATE include)

## Installation
include(GNUInstallDirs)
install(TARGETS franka-interface
  # EXPORT FrankaTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  # RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

