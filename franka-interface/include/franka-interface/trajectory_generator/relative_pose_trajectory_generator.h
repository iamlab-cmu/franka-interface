#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_RELATIVE_POSE_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_RELATIVE_POSE_TRAJECTORY_GENERATOR_H_

#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"

class RelativePoseTrajectoryGenerator : public PoseTrajectoryGenerator {
 public:
  using PoseTrajectoryGenerator::PoseTrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory(const franka::RobotState &robot_state,
                             SkillType skill_type=SkillType::ImpedanceControlSkill) override;

 protected:
  std::array<double, 16> relative_pose_{};
  Eigen::Vector3d relative_position_;
  Eigen::Quaterniond relative_orientation_;
  
};

#endif  // FRANKA_INTERFACE_TRAJECTORY_GENERATOR_RELATIVE_POSE_TRAJECTORY_GENERATOR_H_