#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_MIN_JERK_JOINT_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_MIN_JERK_JOINT_TRAJECTORY_GENERATOR_H_

#include "franka-interface/trajectory_generator/joint_trajectory_generator.h"

class MinJerkJointTrajectoryGenerator : public JointTrajectoryGenerator {
 public:
  using JointTrajectoryGenerator::JointTrajectoryGenerator;

  void get_next_step(const franka::RobotState &robot_state) override;

};

#endif	// FRANKA_INTERFACE_TRAJECTORY_GENERATOR_MIN_JERK_JOINT_TRAJECTORY_GENERATOR_H_