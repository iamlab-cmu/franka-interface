#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_GRIPPER_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_GRIPPER_TRAJECTORY_GENERATOR_H_

#include "franka-interface/trajectory_generator/trajectory_generator.h"

/**
 * Used in Gripper skill. Specifies 3 parameters in the following order
 *
 *      1) gripper width
 *      2) gripper speed
 *      3) gripper force
 */
class GripperTrajectoryGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory(const franka::RobotState &robot_state, SkillType skill_type) {};

  void get_next_step(const franka::RobotState &robot_state) {};

  /**
   * Get width to move the gripper to.
   * @return width
   */
  double getWidth();

  /**
   * Get speed to move the gripper.
   * @return gripper speed.
   */
  double getSpeed();

  /**
   * Get Force to grasp an object.
   * @return gripper force
   */
  double getForce();

  /**
   * Check if the skill requires to grasp the object.
   * @return True if the skill requires to grasp the object, returns false if it does not.
   */
  bool isGraspSkill();

 private:
  GripperTrajectoryGeneratorMessage gripper_trajectory_params_;

  double width_=1.0;
  double speed_=0.0;
  double force_=0.0;
  bool is_grasp_skill_{false};
};

#endif  // FRANKA_INTERFACE_TRAJECTORY_GENERATOR_GRIPPER_TRAJECTORY_GENERATOR_H_
