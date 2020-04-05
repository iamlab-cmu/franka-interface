#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_PASS_THROUGH_FORCE_POSITION_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_PASS_THROUGH_FORCE_POSITION_TRAJECTORY_GENERATOR_H_

#include <Eigen/Dense>
#include "franka-interface/trajectory_generator/trajectory_generator.h"

class PassThroughForcePositionTrajectoryGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  void get_next_step(const franka::RobotState &robot_state) {};

  void parse_parameters() override;

  void initialize_trajectory(const franka::RobotState &robot_state, SkillType skill_type) override;

  void parse_sensor_data(const franka::RobotState &robot_state) override;

  const std::array<double, 16>& get_target_pose() const;

  const std::array<double, 6>& get_target_force() const;

 private:
  RunTimeMessage run_time_params_;
  ForcePositionSensorMessage force_position_sensor_msg_;

  std::array<double, 16> target_pose_{};
  std::array<double, 6> target_force_{};
};

#endif	// FRANKA_INTERFACE_TRAJECTORY_GENERATOR_PASS_THROUGH_FORCE_POSITION_TRAJECTORY_GENERATOR_H_