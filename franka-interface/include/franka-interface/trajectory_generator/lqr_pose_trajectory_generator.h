#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_LQR_POSE_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_LQR_POSE_TRAJECTORY_GENERATOR_H_

#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"
#include "franka-interface/franka_robot.h"
#include<Eigen/StdVector>

class LqrPoseTrajectoryGenerator : public PoseTrajectoryGenerator {
 public:
  using PoseTrajectoryGenerator::PoseTrajectoryGenerator;

  void parse_sensor_data(const franka::RobotState &robot_state) override;
  void get_next_step(const franka::RobotState &robot_state) override;
  void initialize_model(FrankaRobot *robot) override;

  std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > K_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >(int(run_time_/dt_)); // store feedback gains
  Eigen::VectorXd rho;
  Eigen::MatrixXd xf1;
  Eigen::MatrixXd xf2;
  Eigen::VectorXd controlt_;
 private:
  double slerp_t_ = 0.0;

  PosePositionSensorMessage pose_sensor_msg_;
  const franka::Model *model_;
};

#endif	// FRANKA_INTERFACE_TRAJECTORY_GENERATOR_LQR_POSE_TRAJECTORY_GENERATOR_H_