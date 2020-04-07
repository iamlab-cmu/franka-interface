//
// Created by jacky on 1/26/19.
//

#include "franka-interface/trajectory_generator/impulse_trajectory_generator.h"

#include <cassert>
#include <type_traits>
#include <iostream>
#include <memory.h>

#include <franka-interface-common/definitions.h>

void ImpulseTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = impulse_trajectory_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    run_time_ = impulse_trajectory_params_.run_time();
    acc_time_ = impulse_trajectory_params_.acc_time();
    max_translation_ = impulse_trajectory_params_.max_trans();
    max_rotation_ = impulse_trajectory_params_.max_rot();

    for(int i = 0; i < 3; i++) {
      target_force_torque_[i] = impulse_trajectory_params_.forces(i);
      target_force_torque_[i+3] = impulse_trajectory_params_.torques(i);
    }
  } else {
    std::cout << "Parsing ImpulseTrajectoryGenerator params failed. Data size = " << data_size << std::endl;
  }
}

void ImpulseTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state,
                                                       SkillType skill_type) {
  initialize_initial_states(robot_state, skill_type);
}

void ImpulseTrajectoryGenerator::initialize_initial_states(const franka::RobotState &robot_state,
                                                           SkillType skill_type) {
  switch(skill_type) {
    case SkillType::ForceTorqueSkill:
      initial_pose_ = robot_state.O_T_EE;
      break;
    default:
      initial_pose_ = robot_state.O_T_EE;
      std::cout << "Invalid Skill Type provided: " << static_cast<std::underlying_type<SkillType>::type>(skill_type) << "\n";
      std::cout << "Using default O_T_EE" << std::endl;
  }

  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_pose_.data()));
  initial_position_ = Eigen::Vector3d(initial_transform.translation());
  initial_orientation_ = Eigen::Quaterniond(initial_transform.linear());
}

void ImpulseTrajectoryGenerator::get_next_step(const franka::RobotState &robot_state) {
  t_ = time_;

  check_displacement_cap(robot_state);

  double coef = 0.0;
  if (!should_deacc_) {
    if (t_ >= 0 && t_ < acc_time_) {
      coef = t_/acc_time_;
    } else if (t_ >= acc_time_ && t_ < run_time_ - acc_time_) {
      coef = 1.0;
    } else if (t_ >= run_time_ - acc_time_ && t_ < run_time_) {
      coef = (run_time_ - t_)/acc_time_;
    } else {
      coef = 0.0;
    }
  }

  for (size_t i = 0; i < target_force_torque_.size(); i++) {
    desired_force_torque_[i] = coef * target_force_torque_[i];
  }
}

void ImpulseTrajectoryGenerator::check_displacement_cap(const franka::RobotState& robot_state) {
  // check if max translation and rotation caps are reached
  if (!should_deacc_) {
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    
    if (max_translation_ > 0) {
      current_position_ = transform.translation();
      if ((current_position_ - initial_position_).norm() > max_translation_) {
        should_deacc_ = true;
      }
    }

    if (max_rotation_ > 0) {
      current_orientation_ = transform.linear();
      if (current_orientation_.coeffs().dot(initial_orientation_.coeffs()) < 0.0) {
        current_orientation_.coeffs() << -current_orientation_.coeffs();
      }
      Eigen::Quaterniond Q_delta(initial_orientation_ * current_orientation_.inverse());
      Eigen::AngleAxisd A_delta(Q_delta);
      if (A_delta.angle() > max_rotation_) {
        should_deacc_ = true;
      }
    }
  }
}
  
const std::array<double, 6>& ImpulseTrajectoryGenerator::get_desired_force_torque() const {
  return desired_force_torque_;
}

const Eigen::Vector3d& ImpulseTrajectoryGenerator::get_initial_position() const {
  return initial_position_;
}

const Eigen::Quaterniond& ImpulseTrajectoryGenerator::get_initial_orientation() const {
  return initial_orientation_;
}