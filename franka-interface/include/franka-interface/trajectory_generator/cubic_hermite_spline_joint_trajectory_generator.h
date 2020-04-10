#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_CUBIC_HERMITE_SPLINE_JOINT_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_CUBIC_HERMITE_SPLINE_JOINT_TRAJECTORY_GENERATOR_H_

#include "franka-interface/trajectory_generator/joint_trajectory_generator.h"

class CubicHermiteSplineJointTrajectoryGenerator : public JointTrajectoryGenerator {
 public:
  using JointTrajectoryGenerator::JointTrajectoryGenerator;

  void get_next_step(const franka::RobotState &robot_state) override;

  void parse_sensor_data(const franka::RobotState &robot_state) override;

 private:
  double seg_start_time_ = 0.;
  double seg_run_time = 0.;

  std::array<double, 7> initial_joint_velocities_{{0., 0., 0., 0., 0., 0., 0.}};
  std::array<double, 7> goal_joint_velocities_{{0., 0., 0., 0., 0., 0., 0.}};

  JointPositionVelocitySensorMessage joint_sensor_msg_;

};

#endif	// FRANKA_INTERFACE_TRAJECTORY_GENERATOR_CUBIC_HERMITE_SPLINE_JOINT_TRAJECTORY_GENERATOR_H_