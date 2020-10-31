//
// Created by Kevin on 3/25/19.
// From https://mika-s.github.io/python/control-theory/trajectory-generation/2017/12/06/trajectory-generation-with-a-minimum-jerk-trajectory.html
//

#include "franka-interface/trajectory_generator/min_jerk_pose_trajectory_generator.h"

void MinJerkPoseTrajectoryGenerator::get_next_step(const franka::RobotState &robot_state) {
  t_ = std::min(std::max(time_ / run_time_, 0.0), 1.0);
  slerp_t_ = (10 * std::pow(t_, 3) - 15 * std::pow(t_, 4) + 6 * std::pow(t_, 5));
  
  for (int i = 0; i < desired_position_.size(); i++) {
    desired_position_[i] = initial_position_[i] + 
                           (goal_position_[i] - initial_position_[i]) * slerp_t_;
  }

  desired_orientation_ = initial_orientation_.slerp(slerp_t_, goal_orientation_);

  calculate_desired_pose();
}
  