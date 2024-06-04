//
// Created by Kevin on 11/1/22.
//

#include "franka-interface/trajectory_generator/pass_through_joint_velocity_trajectory_generator.h"

#include <cmath>

void PassThroughJointVelocityTrajectoryGenerator::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readTrajectoryGeneratorSensorMessage(joint_velocity_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
    for (int i = 0; i < 7; i++) {
      desired_joint_velocities_[i] = joint_velocity_sensor_msg_.joint_velocities(i);
    }
  }
}
  