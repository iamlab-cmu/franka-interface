#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_LINEAR_FORCE_POSITION_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_LINEAR_FORCE_POSITION_TRAJECTORY_GENERATOR_H_

#include <Eigen/Dense>
#include "franka-interface/trajectory_generator/force_position_trajectory_generator.h"

class LinearForcePositionTrajectoryGenerator : public ForcePositionTrajectoryGenerator {
 public:
  using ForcePositionTrajectoryGenerator::ForcePositionTrajectoryGenerator;

  void get_next_step(const franka::RobotState &robot_state) {};

  void parse_parameters() override;

  void initialize_trajectory(const franka::RobotState &robot_state, SkillType skill_type) override;

  void parse_sensor_data(const franka::RobotState &robot_state) override;

  const std::array<double, 16>& get_desired_pose() override;

  const std::array<double, 6>& get_desired_force() override;

 private:
  double seg_run_time_ = 0.;
  double seg_start_time_;

  Eigen::Affine3d goal_transform_{};
};

#endif	// FRANKA_INTERFACE_TRAJECTORY_GENERATOR_LINEAR_FORCE_POSITION_TRAJECTORY_GENERATOR_H_