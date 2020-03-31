#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_POSE_DMP_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_POSE_DMP_TRAJECTORY_GENERATOR_H_

#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"

class PoseDmpTrajectoryGenerator : public PoseTrajectoryGenerator {
 public:
  using PoseTrajectoryGenerator::PoseTrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory(const franka::RobotState &robot_state,
                             SkillType skill_type=SkillType::CartesianPoseSkill) override;

  void get_next_step(const franka::RobotState &robot_state) override;

  std::array<double, 6> y_={};
  std::array<double, 6> dy_={};

 private:
  PoseDMPTrajectoryGeneratorMessage pose_dmp_trajectory_params_;

  bool orientation_only_ = false;
  bool position_only_ = false;

  // Variables initialized from shared memory should be doubles.
  double alpha_=5.0;
  double beta_=5.0/4.0;
  double tau_=0.0;
  double x_=1.0;
  int num_basis_=40;
  int num_dims_=6;
  int num_sensor_values_=10;
  std::array<double, 40> basis_mean_{};
  std::array<double, 40> basis_std_{};
  std::array<std::array<std::array<double, 40>, 10>, 6> weights_{};
  std::array<double, 10> initial_sensor_values_{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  std::array<double, 6> y0_={};

  void getInitialMeanAndStd();
};

#endif  // FRANKA_INTERFACE_TRAJECTORY_GENERATOR_POSE_DMP_TRAJECTORY_GENERATOR_H_
