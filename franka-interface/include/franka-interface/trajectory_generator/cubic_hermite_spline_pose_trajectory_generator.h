#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_CUBIC_HERMITE_SPLINE_POSE_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_CUBIC_HERMITE_SPLINE_POSE_TRAJECTORY_GENERATOR_H_

#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"

class CubicHermiteSplinePoseTrajectoryGenerator : public PoseTrajectoryGenerator {
 public:
  using PoseTrajectoryGenerator::PoseTrajectoryGenerator;

  void initialize_trajectory(const franka::RobotState &robot_state, SkillType skill_type=SkillType::ImpedanceControlSkill) override;

  void get_next_step(const franka::RobotState &robot_state) override;

  void parse_sensor_data(const franka::RobotState &robot_state) override;

 private:
  double seg_start_time_ = 0.;
  double seg_run_time = 0.;

  std::array<double, 6> initial_pose_velocities_{{0., 0., 0., 0., 0., 0.}};
  std::array<double, 6> goal_pose_velocities_{{0., 0., 0., 0., 0., 0.}};

  Eigen::Vector3d initial_euler_;
  Eigen::Vector3d goal_euler_;
  Eigen::Vector3d desired_euler_;

  PosePositionVelocitySensorMessage pose_sensor_msg_;

};

#endif	// FRANKA_INTERFACE_TRAJECTORY_GENERATOR_CUBIC_HERMITE_SPLINE_POSE_TRAJECTORY_GENERATOR_H_