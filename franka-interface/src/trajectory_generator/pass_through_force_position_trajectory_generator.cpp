//
// Created by jacky on 4/3/20.
//

#include "franka-interface/trajectory_generator/pass_through_force_position_trajectory_generator.h"
#include <cmath>

void PassThroughForcePositionTrajectoryGenerator::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readTrajectoryGeneratorSensorMessage(force_position_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {

    for (size_t i = 0; i < 16; i++) {
      target_pose_[i] = force_position_sensor_msg_.pose(i);
    }
    for (int i = 0; i < 6; i++) {
      target_force_[i] = force_position_sensor_msg_.force(i);
    }
  }
}

void PassThroughForcePositionTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state, SkillType skill_type) {
  for (int i = 0; i < 16; i++) {
    target_pose_[i] = robot_state.O_T_EE[i];
  }
  for (int i = 0; i < 6; i++) {
    target_force_[i] = 0.;
  }
}
  
const std::array<double, 16>& PassThroughForcePositionTrajectoryGenerator::get_target_pose() const {
  return target_pose_;
}

const std::array<double, 6>& PassThroughForcePositionTrajectoryGenerator::get_target_force() const {
  return target_force_;
}