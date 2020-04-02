#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_PASS_THROUGH_JOINT_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_PASS_THROUGH_JOINT_TRAJECTORY_GENERATOR_H_

#include "franka-interface/trajectory_generator/joint_trajectory_generator.h"

class PassThroughJointTrajectoryGenerator : public JointTrajectoryGenerator {
 public:
  using JointTrajectoryGenerator::JointTrajectoryGenerator;

  void get_next_step(const franka::RobotState &robot_state) {};

  void parse_sensor_data(const franka::RobotState &robot_state) override;

 private:
  JointPositionSensorMessage joint_sensor_msg_;

};

#endif	// FRANKA_INTERFACE_TRAJECTORY_GENERATOR_PASS_THROUGH_JOINT_TRAJECTORY_GENERATOR_H_