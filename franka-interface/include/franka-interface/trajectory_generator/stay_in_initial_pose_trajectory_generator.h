#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_STAY_IN_INITIAL_POSE_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_STAY_IN_INITIAL_POSE_TRAJECTORY_GENERATOR_H_

#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"

class StayInInitialPoseTrajectoryGenerator : public PoseTrajectoryGenerator {
 public:
  using PoseTrajectoryGenerator::PoseTrajectoryGenerator;

  void parse_parameters() override;

  void get_next_step(const franka::RobotState &robot_state) override;
  
 protected:
  RunTimeMessage run_time_msg_;
};

#endif	// FRANKA_INTERFACE_TRAJECTORY_GENERATOR_STAY_IN_INITIAL_POSE_TRAJECTORY_GENERATOR_H_