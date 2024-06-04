#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_PASS_THROUGH_CARTESIAN_VELOCITY_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_PASS_THROUGH_CARTESIAN_VELOCITY_TRAJECTORY_GENERATOR_H_

#include "franka-interface/trajectory_generator/cartesian_velocity_trajectory_generator.h"

class PassThroughCartesianVelocityTrajectoryGenerator : public CartesianVelocityTrajectoryGenerator {
 public:
  using CartesianVelocityTrajectoryGenerator::CartesianVelocityTrajectoryGenerator;

  void get_next_step(const franka::RobotState &robot_state) {};

  void parse_sensor_data(const franka::RobotState &robot_state) override;

 private:
  CartesianVelocitySensorMessage cartesian_velocity_sensor_msg_;

};

#endif	// FRANKA_INTERFACE_TRAJECTORY_GENERATOR_PASS_THROUGH_CARTESIAN_VELOCITY_TRAJECTORY_GENERATOR_H_