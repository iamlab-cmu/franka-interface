//
// Created by Kevin on 3/25/19.
// From https://mika-s.github.io/python/control-theory/trajectory-generation/2017/12/06/trajectory-generation-with-a-minimum-jerk-trajectory.html
//

#include "franka-interface/trajectory_generator/min_jerk_pose_trajectory_generator.h"

void MinJerkPoseTrajectoryGenerator::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readTrajectoryGeneratorSensorMessage(pose_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
    for (int i = 0; i < 3; i++) {
      object_position_[i] = pose_sensor_msg_.position(i);
    }

    object_position_[3] = pose_sensor_msg_.quaternion(0);
    object_position_[4] = pose_sensor_msg_.quaternion(1);
    object_position_[5] = pose_sensor_msg_.quaternion(2);
    object_position_[6] = pose_sensor_msg_.quaternion(3);
  }
}

void MinJerkPoseTrajectoryGenerator::get_next_step(const franka::RobotState &robot_state) {
  t_ = std::min(std::max(time_ / run_time_, 0.0), 1.0);
  slerp_t_ = (10 * std::pow(t_, 3) - 15 * std::pow(t_, 4) + 6 * std::pow(t_, 5));
  
  for (int i = 0; i < desired_position_.size(); i++) {
    desired_position_[i] = initial_position_[i] + 
                           (goal_position_[i] - initial_position_[i]) * slerp_t_;
  }

  desired_orientation_ = initial_orientation_.slerp(slerp_t_, goal_orientation_);

  calculate_desired_pose();
}
  