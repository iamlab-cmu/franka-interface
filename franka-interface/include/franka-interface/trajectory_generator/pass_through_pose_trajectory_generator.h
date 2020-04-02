#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_PASS_THROUGH_POSE_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_PASS_THROUGH_POSE_TRAJECTORY_GENERATOR_H_

#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"

class PassThroughPoseTrajectoryGenerator : public PoseTrajectoryGenerator {
 public:
  using PoseTrajectoryGenerator::PoseTrajectoryGenerator;

  void get_next_step(const franka::RobotState &robot_state) override;

  void parse_sensor_data(const franka::RobotState &robot_state) override;

 private:
  PosePositionSensorMessage pose_sensor_msg_;

};

#endif	// FRANKA_INTERFACE_TRAJECTORY_GENERATOR_PASS_THROUGH_POSE_TRAJECTORY_GENERATOR_H_