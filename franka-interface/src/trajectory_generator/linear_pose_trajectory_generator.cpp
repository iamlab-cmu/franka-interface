//
// Created by Kevin on 11/29/18.
//

#include "franka-interface/trajectory_generator/linear_pose_trajectory_generator.h"

void LinearPoseTrajectoryGenerator::get_next_step(const franka::RobotState &robot_state) {
  t_ = std::min(std::max(time_ / run_time_, 0.0), 1.0);

  desired_position_ = initial_position_ + (goal_position_ - initial_position_) * t_;
  desired_orientation_ = initial_orientation_.slerp(t_, goal_orientation_);

  calculate_desired_pose();
}

