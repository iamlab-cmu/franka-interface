//
// Created by jacky on 3/25/20.
// From https://www.rose-hulman.edu/~finn/CCLI/Notes/day09.pdf
//

#include "franka-interface/trajectory_generator/cubic_hermite_spline_pose_trajectory_generator.h"
#include <cmath>

void CubicHermiteSplinePoseTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state,
                                                                   SkillType skill_type) {
  initialize_initial_and_desired_poses(robot_state, skill_type);
  fix_goal_quaternion();

  initial_euler_ = initial_orientation_.toRotationMatrix().eulerAngles(0, 1, 2);
  goal_euler_ = goal_orientation_.toRotationMatrix().eulerAngles(0, 1, 2);
}

void CubicHermiteSplinePoseTrajectoryGenerator::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readTrajectoryGeneratorSensorMessage(pose_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
    for (int i = 0; i < 3; i++) {
      goal_position_[i] = pose_sensor_msg_.position(i);
    }

    goal_orientation_.w() = pose_sensor_msg_.quaternion(0);
    goal_orientation_.x() = pose_sensor_msg_.quaternion(1);
    goal_orientation_.y() = pose_sensor_msg_.quaternion(2);
    goal_orientation_.z() = pose_sensor_msg_.quaternion(3);

    for (int i = 0; i < 6; i++) {
      initial_pose_velocities_[i] = robot_state.O_dP_EE_c[i];
      goal_pose_velocities_[0] = pose_sensor_msg_.pose_velocities(i);
    }
    for (int i = 0; i < 16; i++) {
      initial_pose_[i] = robot_state.O_T_EE[i];
    }
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_pose_.data()));
    initial_position_ = Eigen::Vector3d(initial_transform.translation());
    initial_orientation_ = Eigen::Quaterniond(initial_transform.linear());

    initial_euler_ = initial_orientation_.toRotationMatrix().eulerAngles(0, 1, 2);
    goal_euler_ = goal_orientation_.toRotationMatrix().eulerAngles(0, 1, 2);

    seg_run_time = pose_sensor_msg_.seg_run_time();
    seg_start_time_ = time_;
  }
}

void CubicHermiteSplinePoseTrajectoryGenerator::get_next_step(const franka::RobotState &robot_state) {


  t_ = std::min(std::max((time_ - seg_start_time_) / seg_run_time, 0.0), 1.0);

  double t2 = t_ * t_;
  double t3 = t2 * t_;
    
  double H0 = 1 - 3*t2 + 2*t3;
  double H1 = t_ - 2*t2 + t3;
  double H2 = -t2 + t3;
  double H3 = 3*t2 - 2*t3;
  
  for (size_t i = 0; i < 3; i++) {
    desired_position_[i] = 
    H0 * initial_position_[i] + 
    H1 * initial_pose_velocities_[i] + 
    H2 * goal_pose_velocities_[i] + 
    H3 * goal_position_[i];
  }

  for (size_t i = 0; i < 3; i++) {
    desired_euler_[i] = 
    H0 * initial_euler_[i] + 
    H1 * initial_pose_velocities_[i + 3] + 
    H2 * goal_pose_velocities_[i + 3] + 
    H3 * goal_euler_[i];
  }
  desired_orientation_ = Eigen::AngleAxisd(desired_euler_[0], Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(desired_euler_[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(desired_euler_[2], Eigen::Vector3d::UnitZ());

  calculate_desired_pose();
  
}
  