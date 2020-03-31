//
// Created by kevin on 3/26/19.
//

#include "franka-interface/trajectory_generator/sine_joint_trajectory_generator.h"

#include <cmath>

void SineJointTrajectoryGenerator::get_next_step(const franka::RobotState &robot_state) {
  t_ = std::min(std::max(time_ / run_time_, 0.0), 1.0);
  sine_t_ = ((std::sin((t_ * M_PI) - (M_PI / 2)) + 1) / 2);

  for (size_t i = 0; i < desired_joints_.size(); i++) {
    desired_joints_[i] = initial_joints_[i] + (goal_joints_[i] - initial_joints_[i]) * sine_t_;
  }
}
  