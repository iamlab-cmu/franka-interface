#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_FORCE_POSITION_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_FORCE_POSITION_TRAJECTORY_GENERATOR_H_

#include <Eigen/Dense>
#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"

class ForcePositionTrajectoryGenerator : public PoseTrajectoryGenerator {
 public:
  using PoseTrajectoryGenerator::PoseTrajectoryGenerator;

  virtual const std::array<double, 16>& get_desired_pose() = 0;
  virtual const std::array<double, 6>& get_desired_force() = 0;

 protected:
  RunTimeMessage run_time_params_;
  ForcePositionSensorMessage force_position_sensor_msg_;

  std::array<double, 6> desired_force_{};
};

#endif	// FRANKA_INTERFACE_TRAJECTORY_GENERATOR_FORCE_POSITION_TRAJECTORY_GENERATOR_H_