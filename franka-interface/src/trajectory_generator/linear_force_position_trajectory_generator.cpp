//
// Created by jacky on 4/7/20.
//

#include "franka-interface/trajectory_generator/linear_force_position_trajectory_generator.h"
#include <cmath>

void LinearForcePositionTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = run_time_params_.ParseFromArray(params_ + 5, data_size);

  if (parsed_params){
    run_time_ = run_time_params_.run_time();
  } else {
    std::cout << "Parsing PoseTrajectoryGenerator params failed. Data size = " << data_size << std::endl;
  }
}

void LinearForcePositionTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state, SkillType skill_type) {
  initial_pose_ = robot_state.O_T_EE;
  initial_transform_ = Eigen::Affine3d(Eigen::Matrix4d::Map(initial_pose_.data()));
  initial_position_ = Eigen::Vector3d(initial_transform_.translation());
  initial_orientation_ = Eigen::Quaterniond(initial_transform_.linear());

  goal_pose_ = robot_state.O_T_EE;
  goal_transform_ = Eigen::Affine3d(Eigen::Matrix4d::Map(goal_pose_.data()));
  goal_position_ = Eigen::Vector3d(goal_transform_.translation());
  goal_orientation_ = Eigen::Quaterniond(goal_transform_.linear());

  fix_goal_quaternion();

  for (int i = 0; i < 6; i++) {
    desired_force_[i] = 0.;
  }

  seg_run_time_ = run_time_;
  seg_start_time_ = time_;
}

void LinearForcePositionTrajectoryGenerator::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readTrajectoryGeneratorSensorMessage(force_position_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
    initial_pose_ = robot_state.O_T_EE;
    initial_transform_ = Eigen::Affine3d(Eigen::Matrix4d::Map(initial_pose_.data()));
    initial_position_ = Eigen::Vector3d(initial_transform_.translation());
    initial_orientation_ = Eigen::Quaterniond(initial_transform_.linear());

    for (size_t i = 0; i < 16; i++) {
      goal_pose_[i] = force_position_sensor_msg_.pose(i);
    }
    goal_transform_ = Eigen::Affine3d(Eigen::Matrix4d::Map(goal_pose_.data()));
    goal_position_ = Eigen::Vector3d(goal_transform_.translation());
    goal_orientation_ = Eigen::Quaterniond(goal_transform_.linear());
    fix_goal_quaternion();

    for (int i = 0; i < 6; i++) {
      desired_force_[i] = force_position_sensor_msg_.force(i);
    }

    seg_run_time_ = force_position_sensor_msg_.seg_run_time();
    seg_start_time_ = time_;
  }
}
  
const std::array<double, 16>& LinearForcePositionTrajectoryGenerator::get_desired_pose() {
  double t = std::min(std::max((time_ - seg_start_time_) / seg_run_time_, 0.0), 1.0);
  desired_position_ = initial_position_ + (goal_position_ - initial_position_) * t;
  desired_orientation_ = initial_orientation_.slerp(t, goal_orientation_);
  calculate_desired_pose();

  return desired_pose_;
}

const std::array<double, 6>& LinearForcePositionTrajectoryGenerator::get_desired_force() {
  return desired_force_;
}