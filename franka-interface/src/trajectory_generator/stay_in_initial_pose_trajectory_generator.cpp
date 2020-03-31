//
// Created by Kevin on 11/29/18.
//

#include "franka-interface/trajectory_generator/stay_in_initial_pose_trajectory_generator.h"

#include <iostream>

void StayInInitialPoseTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = run_time_msg_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    run_time_ = run_time_msg_.run_time();

  } else {
    std::cout << "Parsing StayInInitialPoseTrajectoryGenerator params failed. Data size = " << data_size << std::endl;
  }
}

void StayInInitialPoseTrajectoryGenerator::get_next_step(const franka::RobotState &robot_state) {
  desired_position_ = initial_position_;
  desired_orientation_ = initial_orientation_;
}