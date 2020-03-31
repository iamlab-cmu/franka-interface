#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_IMPULSE_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_IMPULSE_TRAJECTORY_GENERATOR_H_

#include <array>
#include <Eigen/Dense>

#include "franka-interface/trajectory_generator/trajectory_generator.h"

class ImpulseTrajectoryGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory(const franka::RobotState &robot_state, SkillType skill_type=SkillType::ForceTorqueSkill) override;

  void initialize_initial_states(const franka::RobotState &robot_state, SkillType skill_type);

  void get_next_step(const franka::RobotState &robot_state) override;

  void check_displacement_cap(const franka::RobotState &robot_state);

  const std::array<double, 6>& get_desired_force_torque() const;

  const Eigen::Vector3d& get_initial_position() const;

  const Eigen::Quaterniond& get_initial_orientation() const;

 private:
  ImpulseTrajectoryGeneratorMessage impulse_trajectory_params_;

  double acc_time_ = 0.0;

  std::array<double, 16> initial_pose_{};

  std::array<double, 6> target_force_torque_{};
  bool should_deacc_ = false;

  double max_translation_{0.0};
  double max_rotation_{0.0}; 

  std::array<double, 6> desired_force_torque_{};
  Eigen::Vector3d initial_position_;
  Eigen::Quaterniond initial_orientation_;

  Eigen::Vector3d current_position_;
  Eigen::Quaterniond current_orientation_;
};

#endif	// FRANKA_INTERFACE_TRAJECTORY_GENERATOR_IMPULSE_TRAJECTORY_GENERATOR_H_