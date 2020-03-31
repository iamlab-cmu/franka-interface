#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_LINEAR_POSE_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_LINEAR_POSE_TRAJECTORY_GENERATOR_H_

#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"

class LinearPoseTrajectoryGenerator : public PoseTrajectoryGenerator {
 public:
  using PoseTrajectoryGenerator::PoseTrajectoryGenerator;

  void get_next_step(const franka::RobotState &robot_state) override;
};

#endif  // FRANKA_INTERFACE_TRAJECTORY_GENERATOR_LINEAR_POSE_TRAJECTORY_GENERATOR_H_