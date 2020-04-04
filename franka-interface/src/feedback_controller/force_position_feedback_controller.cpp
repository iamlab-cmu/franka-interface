//
// Created by jacky on 4/3/20.
//
#include "franka-interface/feedback_controller/force_position_feedback_controller.h"

#include "franka-interface/trajectory_generator/pass_through_force_position_trajectory_generator.h"

void ForcePositionFeedbackController::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = force_position_feedback_params_.ParseFromArray(params_ + 5, data_size);

  if (parsed_params){
    for (int i = 0; i < 6; i++) {
        S_(i, i) = std::min(std::max(force_position_feedback_params_.selection(i), 0.), 1.);
        Sp_(i, i) = 1. - S_(i, i);
    }

    position_kp_ = force_position_feedback_params_.position_kp();
    position_kd_ = 2. * sqrt(position_kp_);

    force_kp_ = force_position_feedback_params_.force_kp();
    force_ki_ = 0.01 * force_kp_;
  } else {
    std::cout << "Parsing ForcePositionFeedbackController params failed. Data size = " << data_size << std::endl;
  }
}

void ForcePositionFeedbackController::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readFeedbackControllerSensorMessage(force_position_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
    for (int i = 0; i < 6; i++) {
        S_(i, i) = fmin(fmax(force_position_sensor_msg_.selection(i), 0.), 1.);
        Sp_(i, i) = 1. - S_(i, i);
    }

    position_kp_ = force_position_sensor_msg_.position_kp();
    position_kd_ = 2. * sqrt(position_kp_);

    force_kp_ = force_position_sensor_msg_.force_kp();
    force_ki_ = 0.01 * force_kp_;
  }
}

void ForcePositionFeedbackController::initialize_controller(FrankaRobot *robot) {
  model_ = robot->getModel();
}

void ForcePositionFeedbackController::get_next_step(const franka::RobotState &robot_state,
                                             TrajectoryGenerator *traj_generator) {
  
  auto fp_traj_generator = dynamic_cast<PassThroughForcePositionTrajectoryGenerator*>(traj_generator);
  if (fp_traj_generator == nullptr) {
    throw std::bad_cast();
  } 

  // Actual transform and force
  Eigen::Affine3d actual_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  std::array<double, 6> actual_force_array = robot_state.O_F_ext_hat_K;
  Eigen::Map<Eigen::VectorXd> actual_force(actual_force_array.data(), 6);

  // Target transform and force
  Eigen::Affine3d target_transform(Eigen::Matrix4d::Map(fp_traj_generator->get_target_pose().data()));
  std::array<double, 6> target_force_array = fp_traj_generator->get_target_force();
  Eigen::Map<Eigen::VectorXd> target_force(target_force_array.data(), 6);

  // Compute errors
  Eigen::Vector3d translation_error = target_transform.translation() - actual_transform.translation();
  Eigen::Quaterniond target_quat(target_transform.linear());
  Eigen::Quaterniond actual_quat(actual_transform.linear());
  Eigen::Quaterniond err_quat(target_quat * actual_quat.inverse());
  Eigen::AngleAxisd err_angle_axis(err_quat);

  // Dynamics
  std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
  std::array<double, 42> jacobian_array = model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
  Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());

  // compute control
  xe_ << translation_error, err_angle_axis.axis() * err_angle_axis.angle();
  fe_ << target_force - actual_force;

  xes_ << S_ * xe_;
  fes_ << Sp_ * fe_;

  q_es_ << jacobian.transpose() * xes_;
  tau_es_ << jacobian.transpose() * fes_;

  tau_p_ << position_kp_ * q_es_ + position_kd_ * (q_es_ - last_q_es_);
  tau_f_ << force_kp_ * tau_es_ + force_ki_ * total_tau_es_;

  tau_task_ << tau_p_ + tau_f_;
  tau_d_ << tau_task_ + coriolis;

  Eigen::VectorXd::Map(&tau_d_array_[0], 7) = tau_d_;

  last_q_es_ << q_es_;
  total_tau_es_ << total_tau_es_ + tau_es_;
}

