#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_QUATERNION_POSE_DMP_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_QUATERNION_POSE_DMP_TRAJECTORY_GENERATOR_H_

#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"

#include <Eigen/Geometry>

class QuaternionPoseDmpTrajectoryGenerator : public PoseTrajectoryGenerator {
 public:
  using PoseTrajectoryGenerator::PoseTrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory(const franka::RobotState &robot_state,
                             SkillType skill_type=SkillType::CartesianPoseSkill) override;

  void get_next_step(const franka::RobotState &robot_state) override;

  void get_next_quaternion_step(const franka::RobotState &robot_state);

  std::array<double, 3> y_={};
  std::array<double, 3> dy_={};
  std::vector<Eigen::Quaternion<double>> q_={};
  std::array<double, 3> dq_={};

 private:
  double quaternion_phase(double curr_t, double alpha, double goal_t, double start_t, double int_dt);
  Eigen::Array3d quaternion_log(const Eigen::Quaterniond& q);
  Eigen::Quaterniond vecExp(const Eigen::Vector3d& input);

  QuaternionPoseDMPTrajectoryGeneratorMessage quat_pose_dmp_trajectory_params_;

  bool ee_frame_ = false;

  // Variables initialized from shared memory should be doubles.
  double alpha_pos_=5.0;
  double beta_pos_=5.0/4.0;
  double tau_pos_=0.0;
  double x_pos_=1.0;
  double alpha_quat_=5.0;
  double beta_quat_=5.0/4.0;
  // This variable is used for calcualating the quaternion phase (canonical variable).
  // TODO(Mohit): Update this to be similar to the position trajectory.
  double alpha_quat_phase_=1.0;
  double tau_quat_=0.0;
  double x_quat_=1.0;
  double curr_time_quat_=0.0;
  double start_time_quat_=0.0;
  double goal_time_quat_=5.0;
  Eigen::Quaterniond goal_quat_;

  int num_basis_pos_=40;
  int num_basis_quat_=40;
  int num_dims_=7;
  int num_sensor_values_pos_=10;
  int num_sensor_values_quat_=10;
  std::array<double, 40> pos_basis_mean_{};
  std::array<double, 40> pos_basis_std_{};
  std::array<std::array<std::array<double, 40>, 10>, 3> pos_weights_{};
  std::array<std::array<double, 10>, 3> pos_initial_sensor_values_{};
  std::array<double, 3> y0_={};

  std::array<double, 40> quat_basis_mean_{};
  std::array<double, 40> quat_basis_std_{};
  std::array<std::array<std::array<double, 40>, 20>, 3> quat_weights_{};
  std::array<std::array<double, 20>, 3> quat_initial_sensor_values_{};
  Eigen::Quaterniond q0_;

  Eigen::Quaterniond last_q_quat_;
  Eigen::Array3d last_qd_;
  Eigen::Array3d last_qdd_;
  Eigen::Quaterniond next_q_quat_;
  Eigen::Array3d next_qd_;
  Eigen::Array3d next_qdd_;

  void getInitialMeanAndStd();
};

#endif  // FRANKA_INTERFACE_TRAJECTORY_GENERATOR_QUATERNION_POSE_DMP_TRAJECTORY_GENERATOR_H_
