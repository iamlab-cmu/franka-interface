//
// Created by kevin on 4/3/18.
//

#include "franka-interface/trajectory_generator/pose_dmp_trajectory_generator.h"

#include <cmath>
#include <iomanip>
#include <iostream>

void PoseDmpTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = pose_dmp_trajectory_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    orientation_only_ = pose_dmp_trajectory_params_.orientation_only();
    position_only_ = pose_dmp_trajectory_params_.position_only();
    ee_frame_ = pose_dmp_trajectory_params_.ee_frame();
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
        initial_sensor_values_[i][j] = pose_dmp_trajectory_params_.initial_sensor_values(i*num_sensor_values_ + j);
      }
    }
  } else {
    std::cout << "Parsing PoseDMPTrajectoryGenerator params failed. Data size = " << data_size << std::endl;
  }
}

void PoseDmpTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state,
                                                        SkillType skill_type) {
  initialize_initial_and_desired_poses(robot_state, skill_type);
  
  Eigen::Matrix3d m = initial_transform_.rotation();

  Eigen::Vector3d ea = m.eulerAngles(0, 1, 2);

  for(size_t i = 0; i < 3; i++) {
    y0_[i] = initial_position_(i);
    y0_[3+i] = ea[i];
    y_[i] = initial_position_(i);
    y_[3+i] = ea[i];
  }
  
  for(size_t i = 0; i < dy_.size(); i++) {
    dy_[i] = robot_state.O_dP_EE_c[i];
  }

  x_ = 1.0;
}

void PoseDmpTrajectoryGenerator::get_next_step(const franka::RobotState &robot_state) {
  static int i, j, k;
  static double ddy;

  static std::array<double, 40> factor{};
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
    factor[k] = (factor[k] * x_) / (den + 1e-8);
  }

  double t = fmin(-log(x_) * 2.0, 1.0);
  factor[0] = pow(t, 3) * (6*pow(t, 2) - 15 * t + 10);

  for (i = 0; i < num_dims_; i++) {
    ddy = (alpha_ * (beta_ * (y0_[i] - y_[i]) - dy_[i] / tau_));
    net_sensor_force = 0;
    for (j = 0; j < num_sensor_values_; j++) {
      sensor_feature = 0;
      for (k=0; k < num_basis_; k++) {
        sensor_feature += (factor[k] * weights_[i][j][k]);
      }
      net_sensor_force += (initial_sensor_values_[i][j] * sensor_feature);
    }
    ddy += (alpha_ * beta_ * net_sensor_force);
    ddy *= (tau_ * tau_);
    dy_[i] += (ddy * dt_);
    y_[i] += (dy_[i] * dt_);
  }

  // Update canonical system.
  x_ -= (x_ * tau_) * dt_;

  if(ee_frame_){
    Eigen::Matrix3d R0 = initial_orientation_.toRotationMatrix();
    Eigen::Matrix<double,3,1>pos;
    Eigen::Matrix<double,3,1>new_pos; 
    Eigen::Matrix3d R_dmp_to_EE;     
    // R_dmp_to_EE << 1,0,0,0,-1,0,0,0,-1;
    R_dmp_to_EE << 0,1,0,1,0,0,0,0,-1;

    pos[0]=y_[0];
    pos[1]=y_[1];
    pos[2]=y_[2];  

    new_pos=R0 * (R_dmp_to_EE * (pos-initial_position_)) + initial_position_;

    desired_position_(0) = new_pos[0];
    desired_position_(1) = new_pos[1];
    desired_position_(2) = new_pos[2];
  }
  else {
    // Finally set the position we want.
    desired_position_(0) = y_[0];
    desired_position_(1) = y_[1];
    desired_position_(2) = y_[2];
  }

  Eigen::Matrix3d n;
  n = Eigen::AngleAxisd(y_[3], Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(y_[4], Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(y_[5], Eigen::Vector3d::UnitZ());

  desired_pose_[0] = n(0,0);
  desired_pose_[1] = n(1,0);
  desired_pose_[2] = n(2,0);
  desired_pose_[4] = n(0,1);
  desired_pose_[5] = n(1,1);
  desired_pose_[6] = n(2,1);
  desired_pose_[8] = n(0,2);
  desired_pose_[9] = n(1,2);
  desired_pose_[10] = n(2,2);
  // The following is done in calculate_desired_position()
  // desired_pose_[12] = desired_position_(0);
  // desired_pose_[13] = desired_position_(1);
  // desired_pose_[14] = desired_position_(2);

  desired_orientation_ = Eigen::Quaterniond(n);

  calculate_desired_position();

}

void PoseDmpTrajectoryGenerator::getInitialMeanAndStd() {
  std::array<double, 40> basis_mean{};
  std::array<double, 40> basis_std{};
  for (int i = 0; i < num_basis_; i++)  {
    basis_mean[i] = exp(-(i)*0.5/(num_basis_-1));
  }
  for (int i = 0; i < num_basis_ - 1; i++) {
    basis_std[i] = 0.5 / (0.65 * pow(basis_mean[i+1] - basis_mean[i], 2));
  }
  basis_std[num_basis_ - 1] = basis_std[num_basis_ - 2];
}
