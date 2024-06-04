#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_CARTESIAN_VELOCITY_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_CARTESIAN_VELOCITY_TRAJECTORY_GENERATOR_H_

#include <array>

#include "franka-interface/trajectory_generator/trajectory_generator.h"

class CartesianVelocityTrajectoryGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory(const franka::RobotState &robot_state, SkillType skill_type=SkillType::CartesianVelocitySkill) override;

  /**
   * Initialize initial and desired Cartesian velocities from robot state
   */
  void initialize_initial_cartesian_velocities(const franka::RobotState &robot_state, SkillType skill_type);

  /**
   * Returns the desired Cartesian velocities. This method is called at every time step of the control loop to
   * find the Cartesian velocities to send at the next step.
   */
  const std::array<double, 6>& get_desired_cartesian_velocities() const;

 protected:
  CartesianVelocityTrajectoryGeneratorMessage cartesian_velocity_trajectory_params_;

  std::array<double, 6> initial_cartesian_velocities_{};
  std::array<double, 6> cartesian_accelerations_{};
  std::array<double, 6> desired_cartesian_velocities_{};
  std::array<double, 6> goal_cartesian_velocities_{};
};

#endif	// FRANKA_INTERFACE_TRAJECTORY_GENERATOR_CARTESIAN_VELOCITY_TRAJECTORY_GENERATOR_H_