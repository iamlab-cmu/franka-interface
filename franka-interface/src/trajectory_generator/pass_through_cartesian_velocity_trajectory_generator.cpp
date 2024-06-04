//
// Created by Kevin on 11/1/22.
//

#include "franka-interface/trajectory_generator/pass_through_cartesian_velocity_trajectory_generator.h"

#include <cmath>

void PassThroughCartesianVelocityTrajectoryGenerator::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readTrajectoryGeneratorSensorMessage(cartesian_velocity_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
    for (int i = 0; i < 6; i++) {
      desired_cartesian_velocities_[i] = cartesian_velocity_sensor_msg_.cartesian_velocities(i);
    }
  }
}
  