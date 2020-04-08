//
// Created by jacky on 4/3/20.
//

#include "franka-interface/trajectory_generator/pass_through_force_position_trajectory_generator.h"
#include <cmath>

void PassThroughForcePositionTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = run_time_params_.ParseFromArray(params_ + 5, data_size);

  if (parsed_params){
    run_time_ = run_time_params_.run_time();
  } else {
    std::cout << "Parsing PoseTrajectoryGenerator params failed. Data size = " << data_size << std::endl;
  }
}

void PassThroughForcePositionTrajectoryGenerator::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readTrajectoryGeneratorSensorMessage(force_position_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {

    for (size_t i = 0; i < 16; i++) {
      desired_pose_[i] = force_position_sensor_msg_.pose(i);
    }
    for (int i = 0; i < 6; i++) {
      desired_force_[i] = force_position_sensor_msg_.force(i);
    }
  }
}

void PassThroughForcePositionTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state, SkillType skill_type) {
  for (int i = 0; i < 16; i++) {
    desired_pose_[i] = robot_state.O_T_EE[i];
  }
  for (int i = 0; i < 6; i++) {
    desired_force_[i] = 0.;
  }
}
  
const std::array<double, 16>& PassThroughForcePositionTrajectoryGenerator::get_desired_pose() {
  return desired_pose_;
}

const std::array<double, 6>& PassThroughForcePositionTrajectoryGenerator::get_desired_force() {
  return desired_force_;
}