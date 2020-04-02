//
// Created by jacky on 4/1/20.
//

#include "franka-interface/trajectory_generator/pass_through_pose_trajectory_generator.h"
#include <cmath>

void PassThroughPoseTrajectoryGenerator::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readTrajectoryGeneratorSensorMessage(pose_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
    for (int i = 0; i < 3; i++) {
      desired_position_[i] = pose_sensor_msg_.position(i);
    }

    desired_orientation_.w() = pose_sensor_msg_.quaternion(0);
    desired_orientation_.x() = pose_sensor_msg_.quaternion(1);
    desired_orientation_.y() = pose_sensor_msg_.quaternion(2);
    desired_orientation_.z() = pose_sensor_msg_.quaternion(3);
  }
}

void PassThroughPoseTrajectoryGenerator::get_next_step(const franka::RobotState &robot_state) {
  calculate_desired_pose();
}
