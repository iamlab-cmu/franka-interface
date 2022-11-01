#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_JOINT_VELOCITY_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_JOINT_VELOCITY_TRAJECTORY_GENERATOR_H_

#include <array>

#include "franka-interface/trajectory_generator/trajectory_generator.h"

class JointVelocityTrajectoryGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory(const franka::RobotState &robot_state, SkillType skill_type=SkillType::JointVelocitySkill) override;

  /**
   * Initialize initial and desired joint velocities from robot state
   */
  void initialize_initial_joint_velocities(const franka::RobotState &robot_state, SkillType skill_type);

  /**
   * Returns the desired joint velocities. This method is called at every time step of the control loop to
   * find the joint velocities to send at the next step.
   */
  const std::array<double, 7>& get_desired_joint_velocities() const;

 protected:
  JointVelocityTrajectoryGeneratorMessage joint_velocity_trajectory_params_;

  std::array<double, 7> initial_joint_velocities_{};
  std::array<double, 7> joint_accelerations_{};
  std::array<double, 7> desired_joint_velocities_{};
  std::array<double, 7> goal_joint_velocities_{};
};

#endif	// FRANKA_INTERFACE_TRAJECTORY_GENERATOR_JOINT_VELOCITY_TRAJECTORY_GENERATOR_H_