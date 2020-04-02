//
// Created by jacky on 3/25/20.
// From https://www.rose-hulman.edu/~finn/CCLI/Notes/day09.pdf
//

#include "franka-interface/trajectory_generator/cubic_hermite_spline_joint_trajectory_generator.h"

#include <cmath>

void CubicHermiteSplineJointTrajectoryGenerator::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readTrajectoryGeneratorSensorMessage(joint_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
    for (int i = 0; i < 7; i++) {
      goal_joints_[i] = joint_sensor_msg_.joints(i);
      goal_joint_velocities_[i] = joint_sensor_msg_.joint_vels(i);
      initial_joints_[i] = robot_state.q[i];
      initial_joint_velocities_[i] = robot_state.dq[i];
    }

    seg_run_time = joint_sensor_msg_.seg_run_time();
    seg_start_time_ = time_;
  }
}

void CubicHermiteSplineJointTrajectoryGenerator::get_next_step(const franka::RobotState &robot_state) {
  t_ = std::min(std::max((time_ - seg_start_time_) / seg_run_time, 0.0), 1.0);

  double t2 = t_ * t_;
  double t3 = t2 * t_;
    
  double H0 = 1 - 3*t2 + 2*t3;
  double H1 = t_ - 2*t2 + t3;
  double H2 = -t2 + t3;
  double H3 = 3*t2 - 2*t3;
  
  for (size_t i = 0; i < desired_joints_.size(); i++) {
    desired_joints_[i] = 
    H0 * initial_joints_[i] + 
    H1 * initial_joint_velocities_[i] + 
    H2 * goal_joint_velocities_[i] + 
    H3 * goal_joints_[i];
  }
}
  