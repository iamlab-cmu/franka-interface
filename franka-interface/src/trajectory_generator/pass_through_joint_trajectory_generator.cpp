//
// Created by jacky on 4/1/20.
//

#include "franka-interface/trajectory_generator/pass_through_joint_trajectory_generator.h"

#include <cmath>

void PassThroughJointTrajectoryGenerator::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readTrajectoryGeneratorSensorMessage(joint_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
    for (int i = 0; i < 7; i++) {
      desired_joints_[i] = joint_sensor_msg_.joints(i);
    }
  }
}
  