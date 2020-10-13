#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_LQR_POSE_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_LQR_POSE_TRAJECTORY_GENERATOR_H_

#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"

#include<Eigen/StdVector>

class LqrPoseTrajectoryGenerator : public PoseTrajectoryGenerator {
 public:
  using PoseTrajectoryGenerator::PoseTrajectoryGenerator;

  void get_next_step(const franka::RobotState &robot_state) override;

  std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > K_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >(int(run_time_/dt_)); // store feedback gains
  
 private:
  double slerp_t_ = 0.0;
};

#endif	// FRANKA_INTERFACE_TRAJECTORY_GENERATOR_LQR_POSE_TRAJECTORY_GENERATOR_H_