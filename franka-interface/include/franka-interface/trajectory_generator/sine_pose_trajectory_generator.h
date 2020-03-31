#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_SINE_POSE_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_SINE_POSE_TRAJECTORY_GENERATOR_H_

#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"

class SinePoseTrajectoryGenerator : public PoseTrajectoryGenerator {
 public:
  using PoseTrajectoryGenerator::PoseTrajectoryGenerator;

  void get_next_step(const franka::RobotState &robot_state) override;
  
 private:
  double sine_t_ = 0.0;
};

#endif  // FRANKA_INTERFACE_TRAJECTORY_GENERATOR_SINE_POSE_TRAJECTORY_GENERATOR_H_