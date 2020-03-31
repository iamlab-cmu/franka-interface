#include "franka-interface/trajectory_generator/relative_pose_trajectory_generator.h"

#include <iostream>

void RelativePoseTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = pose_trajectory_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    run_time_ = pose_trajectory_params_.run_time();

    if(pose_trajectory_params_.pose_size() == 16){
      for(size_t i = 0; i < relative_pose_.size(); i++) {
        relative_pose_[i] = pose_trajectory_params_.pose(i);
      }

      Eigen::Affine3d goal_transform(Eigen::Matrix4d::Map(relative_pose_.data()));
      relative_position_ = Eigen::Vector3d(goal_transform.translation());
      relative_orientation_ = Eigen::Quaterniond(goal_transform.linear());
    } else {

      for(int i = 0; i < 3; i++) {
        relative_position_[i] = pose_trajectory_params_.position(i);
      }

      relative_orientation_.w() = pose_trajectory_params_.quaternion(0);
      relative_orientation_.x() = pose_trajectory_params_.quaternion(1);
      relative_orientation_.y() = pose_trajectory_params_.quaternion(2);
      relative_orientation_.z() = pose_trajectory_params_.quaternion(3);
    }
  } else {
    std::cout << "Parsing RelativePoseTrajectoryGenerator params failed. Data size = " << data_size << std::endl;
  }
}

void RelativePoseTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state,
                                                            SkillType skill_type) {
  initialize_initial_and_desired_poses(robot_state, skill_type);
  goal_position_ = initial_position_ + relative_position_;
  goal_orientation_ = initial_orientation_ * relative_orientation_;

  fix_goal_quaternion();
}