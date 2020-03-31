//
// Created by Kevin on 4/1/19.
//

#include "franka-interface/trajectory_generator/stay_in_initial_joints_trajectory_generator.h"

#include <iostream>

void StayInInitialJointsTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = run_time_msg_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    run_time_ = run_time_msg_.run_time();

  } else {
    std::cout << "Parsing StayInInitialJointsTrajectoryGenerator params failed. Data size = " << data_size << std::endl;
  }
}

void StayInInitialJointsTrajectoryGenerator::get_next_step(const franka::RobotState &robot_state) {
  desired_joints_ = initial_joints_;
}