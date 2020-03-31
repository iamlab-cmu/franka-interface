#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_MIN_JERK_POSE_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_MIN_JERK_POSE_TRAJECTORY_GENERATOR_H_

#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"

class MinJerkPoseTrajectoryGenerator : public PoseTrajectoryGenerator {
 public:
  using PoseTrajectoryGenerator::PoseTrajectoryGenerator;

  void get_next_step(const franka::RobotState &robot_state) override;

 private:
  double slerp_t_ = 0.0;
};

#endif	// FRANKA_INTERFACE_TRAJECTORY_GENERATOR_MIN_JERK_POSE_TRAJECTORY_GENERATOR_H_