//
// Created by kevin on 4/3/18.
//

#include "franka-interface/trajectory_generator/goal_pose_dmp_trajectory_generator.h"

#include <cmath>
#include <iostream>

void GoalPoseDmpTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = pose_dmp_trajectory_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    orientation_only_ = pose_dmp_trajectory_params_.orientation_only();
    position_only_ = pose_dmp_trajectory_params_.position_only();
    run_time_ = pose_dmp_trajectory_params_.run_time();
    tau_ = pose_dmp_trajectory_params_.tau();
    alpha_ = pose_dmp_trajectory_params_.alpha();
    beta_ = pose_dmp_trajectory_params_.beta();
    num_basis_ = pose_dmp_trajectory_params_.num_basis();
    num_sensor_values_ = pose_dmp_trajectory_params_.num_sensor_values();

    if(orientation_only_ || position_only_){
      num_dims_ = 3;
    }

    for (int i = 0; i < num_basis_; i++) {
      basis_mean_[i] = pose_dmp_trajectory_params_.basis_mean(i);
      basis_std_[i] = pose_dmp_trajectory_params_.basis_std(i);
    }

    for (int i = 0; i < num_dims_; i++) {
      for (int j = 0; j < num_sensor_values_; j++) {
        for (int k = 0; k < num_basis_; k++) {
          int index = (i * num_sensor_values_* num_basis_) + (j * num_basis_) + k;
          weights_[i][j][k] = pose_dmp_trajectory_params_.weights(index);
        }
      }
    }

    for (int i = 0; i < num_dims_; i++) {
      for (int j = 0; j < num_sensor_values_; j++) {
        int index = (i * num_sensor_values_) + j;
        initial_sensor_values_[i][j] = pose_dmp_trajectory_params_.initial_sensor_values(index);
      }
    }

  } else {
    std::cout << "Parsing GoalPoseDMPTrajectoryGenerator params failed. Data size = " << data_size << std::endl;
  }
}

void GoalPoseDmpTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state,
                                                        SkillType skill_type) {
  initialize_initial_and_desired_poses(robot_state, skill_type);
  
  for(size_t i = 0; i < y0_.size(); i++) {
    y0_[i] = initial_position_(i);
  }
  for(size_t i = 0; i < y_.size(); i++) {
    y_[i] = initial_position_(i);
  }
  for(size_t i = 0; i < dy_.size(); i++) {
    dy_[i] = robot_state.O_dP_EE_c[i];
  }

  x_ = 1.0;

  double dmp_z_dist = 0.0;

  if(initial_sensor_values_[2][0] >= -0.01) {
    dmp_z_dist = initial_sensor_values_[2][0];
  } else {
    dmp_z_dist = initial_sensor_values_[2][0] / 2;
  }

  if(min_z > initial_position_(2) + eps){
    for (int j = 0; j < num_sensor_values_; j++) {
      initial_sensor_values_[2][j] = 0.0;
    }
  } else if(min_z > initial_position_(2) + dmp_z_dist){
    initial_sensor_values_[2][0] = (initial_position_(2) - min_z) * 2;
  }
}

void GoalPoseDmpTrajectoryGenerator::get_next_step(const franka::RobotState &robot_state) {
  static int i, j, k;
  static double ddy, t;

  static std::array<double, 20> factor{};
  static double den = 0.;
  static double net_sensor_force;
  static double sensor_feature;

  // Calculate feature values.
  den = 0.;
  for (k = 0; k < num_basis_; k++) {
    factor[k] = exp(-basis_std_[k] * pow((x_ - basis_mean_[k]), 2));
    // first basis is for the min-jerk feature
    if (k > 0) {
      den += factor[k];
    }
  }

  for (k = 1; k < num_basis_; k++) {
    // factor[k] = (factor[k] * x_) / (den * basis_mean_[k]);
    factor[k] = (factor[k] * x_) / (den + 1e-8);
  }
  t = fmin(-log(x_)/tau_, 1);
  factor[0] = pow(t, 3) * (6*pow(t, 2) - 15 * t + 10);

  for (i = 0; i < num_dims_; i++) {
    ddy = (alpha_ * (beta_ * (y0_[i] - y_[i]) - tau_ * dy_[i]));
    net_sensor_force = 0;
    for (j = 0; j < num_sensor_values_; j++) {
      sensor_feature = 0;
      for (k=0; k < num_basis_; k++) {
        sensor_feature += (factor[k] * weights_[i][j][k]);
      }
      net_sensor_force += (initial_sensor_values_[i][j] * sensor_feature);
    }
    ddy += (alpha_ * beta_ * net_sensor_force);
    ddy /= (tau_ * tau_);
    // NOTE: dt_ used below can sometimes be greater than 0.001 
    // (e.g. 0.003, 0.006), we believe this might be because of some 
    // low-pass filtering that franka implements. We should theoretically 
    // use a fixed dt, but this works fine for now.
    dy_[i] += (ddy * dt_);
    y_[i] += (dy_[i] * dt_);
  }

  // Update canonical system.
  x_ -= (x_ * tau_) * dt_;

  // Finally set the position we want.
  desired_position_(0) = y_[0];
  desired_position_(1) = y_[1];
  desired_position_(2) = y_[2];

  calculate_desired_position();
}

void GoalPoseDmpTrajectoryGenerator::getInitialMeanAndStd() {
  std::array<double, 10> basis_mean{};
  std::array<double, 10> basis_std{};
  for (int i = 0; i < num_basis_; i++)  {
    basis_mean[i] = exp(-(i)*0.5/(num_basis_-1));
  }
  for (int i = 0; i < num_basis_ - 1; i++) {
    basis_std[i] = 0.5 / (0.65 * pow(basis_mean[i+1] - basis_mean[i], 2));
  }
  basis_std[num_basis_ - 1] = basis_std[num_basis_ - 2];
}