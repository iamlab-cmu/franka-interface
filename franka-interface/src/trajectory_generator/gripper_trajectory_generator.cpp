//
// Created by iamlab on 12/1/18.
//

#include "franka-interface/trajectory_generator/gripper_trajectory_generator.h"

#include <iostream>

void GripperTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = gripper_trajectory_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    is_grasp_skill_ = gripper_trajectory_params_.grasp();

    width_ = gripper_trajectory_params_.width();
    speed_ = gripper_trajectory_params_.speed();

    if(is_grasp_skill_){
      force_ = gripper_trajectory_params_.force();
    }
  } else {
    std::cout << "Parsing GripperTrajectoryGenerator params failed. Data size = " << data_size << std::endl;
  }
}

double GripperTrajectoryGenerator::getWidth() {
  return width_;
}

double GripperTrajectoryGenerator::getSpeed()  {
  return speed_;
}

double GripperTrajectoryGenerator::getForce()  {
  return force_;
}

bool GripperTrajectoryGenerator::isGraspSkill(){
  return is_grasp_skill_;
}
