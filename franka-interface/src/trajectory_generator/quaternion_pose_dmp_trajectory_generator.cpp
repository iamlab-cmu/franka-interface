#include "franka-interface/trajectory_generator/quaternion_pose_dmp_trajectory_generator.h"

#include <cmath>
#include <iomanip>
#include <iostream>

void QuaternionPoseDmpTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = quat_pose_dmp_trajectory_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    ee_frame_ = quat_pose_dmp_trajectory_params_.ee_frame();
    run_time_ = quat_pose_dmp_trajectory_params_.run_time();

    tau_pos_ = quat_pose_dmp_trajectory_params_.tau_pos();
    alpha_pos_ = quat_pose_dmp_trajectory_params_.alpha_pos();
    beta_pos_ = quat_pose_dmp_trajectory_params_.beta_pos();
    tau_quat_ = quat_pose_dmp_trajectory_params_.tau_quat();
    alpha_quat_ = quat_pose_dmp_trajectory_params_.alpha_quat();
    beta_quat_ = quat_pose_dmp_trajectory_params_.beta_quat();

    num_basis_pos_ = quat_pose_dmp_trajectory_params_.num_basis_pos();
    num_basis_quat_ = quat_pose_dmp_trajectory_params_.num_basis_quat();
    num_sensor_values_pos_ = quat_pose_dmp_trajectory_params_.num_sensor_values_pos();
    num_sensor_values_quat_ = quat_pose_dmp_trajectory_params_.num_sensor_values_quat();

    for (int i = 0; i < num_basis_pos_; i++) {
      pos_basis_mean_[i] = quat_pose_dmp_trajectory_params_.pos_basis_mean(i);
      pos_basis_std_[i] = quat_pose_dmp_trajectory_params_.pos_basis_std(i);
    }
    for (int i = 0; i < num_basis_quat_; i++) {
      quat_basis_mean_[i] = quat_pose_dmp_trajectory_params_.quat_basis_mean(i);
      quat_basis_std_[i] = quat_pose_dmp_trajectory_params_.quat_basis_std(i);
    }

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < num_sensor_values_pos_; j++) {
        for (int k = 0; k < num_basis_pos_; k++) {
          int index = (i * num_sensor_values_pos_* num_basis_pos_) + (j * num_basis_pos_) + k;
          pos_weights_[i][j][k] = quat_pose_dmp_trajectory_params_.pos_weights(index);
        }
      }
    }
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < num_sensor_values_quat_; j++) {
        for (int k = 0; k < num_basis_quat_; k++) {
          int index = (i * num_sensor_values_quat_ * num_basis_quat_) + (j * num_basis_quat_) + k;
          quat_weights_[i][j][k] = quat_pose_dmp_trajectory_params_.quat_weights(index);
        }
      }
    }

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < num_sensor_values_pos_; j++) {
        pos_initial_sensor_values_[i][j] = quat_pose_dmp_trajectory_params_.pos_initial_sensor_values(i*num_sensor_values_pos_ + j);
      }
    }

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < num_sensor_values_quat_; j++) {
        quat_initial_sensor_values_[i][j] = quat_pose_dmp_trajectory_params_.quat_initial_sensor_values(i*num_sensor_values_quat_ + j);
      }
    }
  } else {
    std::cout << "Parsing PoseDMPTrajectoryGenerator params failed. Data size = " << data_size << std::endl;
  }
}

void QuaternionPoseDmpTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state,
                                                                 SkillType skill_type) {
  initialize_initial_and_desired_poses(robot_state, skill_type);
  
  Eigen::Matrix3d m = initial_transform_.rotation();
  Eigen::Quaterniond q(m);
  q.normalize();

  for(size_t i = 0; i < 3; i++) {
    y0_[i] = initial_position_(i);
    y_[i] = initial_position_(i);
  }
  q0_ = Eigen::Quaterniond(m);
  q_.clear();
  q_.push_back(q);

  for(size_t i = 0; i < dy_.size(); i++) {
    dy_[i] = robot_state.O_dP_EE_c[i];
  }

  x_pos_ = 1.0;
  x_quat_ = 1.0;
}

void QuaternionPoseDmpTrajectoryGenerator::get_next_step(const franka::RobotState &robot_state) {
  static int i, j, k;
  static double ddy;

  static std::array<double, 40> factor{};
  static double den = 0.;
  static double net_sensor_force;
  static double sensor_feature;

  // Calculate feature values.
  den = 0.;
  for (k = 0; k < num_basis_pos_; k++) {
    factor[k] = exp(-pos_basis_std_[k] * pow((x_pos_ - pos_basis_mean_[k]), 2));
    // first basis is for the min-jerk feature
    if (k > 0) {
      den += factor[k];
    }
  }

  for (k = 1; k < num_basis_pos_; k++) {
    factor[k] = (factor[k] * x_pos_) / (den + 1e-8);
  }

  double t = fmin(-log(x_pos_) * 2.0, 1.0);
  factor[0] = pow(t, 3) * (6*pow(t, 2) - 15 * t + 10);

  for (i = 0; i < 3; i++) {
    ddy = (alpha_pos_ * (beta_pos_ * (y0_[i] - y_[i]) - dy_[i] / tau_pos_));
    net_sensor_force = 0;
    for (j = 0; j < num_sensor_values_pos_; j++) {
      sensor_feature = 0;
      for (k=0; k < num_basis_pos_; k++) {
        sensor_feature += (factor[k] * pos_weights_[i][j][k]);
      }
      net_sensor_force += (pos_initial_sensor_values_[i][j] * sensor_feature);
    }
    ddy += (alpha_pos_ * beta_pos_ * net_sensor_force);
    ddy *= (tau_pos_ * tau_pos_);
    dy_[i] += (ddy * dt_);
    y_[i] += (dy_[i] * dt_);
  }

  // Update canonical system.
  x_pos_ -= (x_pos_ * tau_pos_) * dt_;

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

void QuaternionPoseDmpTrajectoryGenerator::getInitialMeanAndStd() { }
