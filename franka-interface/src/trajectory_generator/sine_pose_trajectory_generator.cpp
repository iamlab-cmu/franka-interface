//
// Created by Kevin on 11/29/18.
//

#include "franka-interface/trajectory_generator/sine_pose_trajectory_generator.h"

void SinePoseTrajectoryGenerator::get_next_step(const franka::RobotState &robot_state) {
  t_ = std::min(std::max(time_ / run_time_, 0.0), 1.0);
  sine_t_ = ((std::sin((t_ * M_PI) - (M_PI / 2)) + 1) / 2);
  
  desired_position_ = initial_position_ + (goal_position_ - initial_position_) * sine_t_;
  desired_orientation_ = initial_orientation_.slerp(sine_t_, goal_orientation_);

  calculate_desired_pose();
}

