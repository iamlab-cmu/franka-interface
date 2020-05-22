//
// Created by jacky on 4/3/20.
//
#include "franka-interface/feedback_controller/force_position_feedback_controller.h"

#include "franka-interface/trajectory_generator/force_position_trajectory_generator.h"

void ForcePositionFeedbackController::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = force_position_feedback_params_.ParseFromArray(params_ + 5, data_size);

  if (parsed_params){
    for (int i = 0; i < 6; i++) {
        S_(i, i) = std::min(std::max(force_position_feedback_params_.selection(i), 0.), 1.);
        Sp_(i, i) = 1. - S_(i, i);
    }
    if (force_position_feedback_params_.error_frame_size() == 9) {
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          R_err(i, j) = force_position_sensor_msg_.error_frame(i * 3 + j);
        }
      }
    }
    use_cartesian_gains_ = force_position_feedback_params_.use_cartesian_gains();
    if (use_cartesian_gains_) {
      for (int i = 0; i < 6; i++) {
        position_kps_cart_(i, i) = force_position_feedback_params_.position_kps_cart(i);
        if (force_position_feedback_params_.position_kds_cart_size() == 6) {
          position_kds_cart_(i, i) = force_position_feedback_params_.position_kds_cart(i);
        } else {
          position_kds_cart_(i, i) = 2. * sqrt(position_kps_cart_(i, i));
        }

        force_kps_cart_(i, i) = force_position_feedback_params_.force_kps_cart(i);
        if (force_position_feedback_params_.force_kis_cart_size() == 6) {
          force_kis_cart_(i, i) = force_position_feedback_params_.force_kis_cart(i);
        } else {
          force_kis_cart_(i, i) = 0.01 * force_kps_cart_(i, i);
        }
      }
    } else {
      for (int i = 0; i < 7; i++) {
        position_kps_joint_(i, i) = force_position_feedback_params_.position_kps_joint(i);
        if (force_position_feedback_params_.position_kds_joint_size() == 7) {
          position_kds_joint_(i, i) = force_position_feedback_params_.position_kds_joint(i);
        } else {
          position_kds_joint_(i, i) = 2. * sqrt(position_kps_joint_(i, i));
        }

        force_kps_joint_(i, i) = force_position_feedback_params_.force_kps_joint(i);
        if (force_position_feedback_params_.force_kis_cart_size() == 7) {
          force_kis_joint_(i, i) = force_position_feedback_params_.force_kis_joint(i);
        } else {
          force_kis_joint_(i, i) = 0.01 * force_kps_joint_(i, i);
        }
      }
    }
  } else {
    std::cout << "Parsing ForcePositionFeedbackController params failed. Data size = " << data_size << std::endl;
  }
}

void ForcePositionFeedbackController::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readFeedbackControllerSensorMessage(force_position_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
    for (int i = 0; i < 6; i++) {
        S_(i, i) = std::min(std::max(force_position_sensor_msg_.selection(i), 0.), 1.);
        Sp_(i, i) = 1. - S_(i, i);
    }
    if (force_position_sensor_msg_.error_frame_size() == 9) {
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          R_err(i, j) = force_position_sensor_msg_.error_frame(i * 3 + j);
        }
      }
    }
    if (use_cartesian_gains_) {
      for (int i = 0; i < 6; i++) {
        position_kps_cart_(i, i) = force_position_sensor_msg_.position_kps_cart(i);
        if (force_position_sensor_msg_.position_kds_cart_size() == 6) {
          position_kds_cart_(i, i) = force_position_sensor_msg_.position_kds_cart(i);
        } else {
          position_kds_cart_(i, i) = 2. * sqrt(position_kps_cart_(i, i));
        }

        force_kps_cart_(i, i) = force_position_sensor_msg_.force_kps_cart(i);
        if (force_position_sensor_msg_.force_kis_cart_size() == 6) {
          force_kis_cart_(i, i) = force_position_sensor_msg_.force_kis_cart(i);
        } else {
          force_kis_cart_(i, i) = 0.01 * force_kps_cart_(i, i);
        }
      }
    } else {
      for (int i = 0; i < 7; i++) {
        position_kps_joint_(i, i) = force_position_sensor_msg_.position_kps_joint(i);
        if (force_position_sensor_msg_.position_kds_joint_size() == 7) {
          position_kds_joint_(i, i) = force_position_sensor_msg_.position_kds_joint(i);
        } else {
          position_kds_joint_(i, i) = 2. * sqrt(position_kps_joint_(i, i));
        }

        force_kps_joint_(i, i) = force_position_sensor_msg_.force_kps_joint(i);
        if (force_position_sensor_msg_.force_kis_cart_size() == 7) {
          force_kis_joint_(i, i) = force_position_sensor_msg_.force_kis_joint(i);
        } else {
          force_kis_joint_(i, i) = 0.01 * force_kps_joint_(i, i);
        }
      }
    }

    if (force_position_sensor_msg_.has_reset_force_integral_error() && 
        force_position_sensor_msg_.reset_force_integral_error()) {
      total_fes_ << total_fes_ * 0;
      total_tau_es_ << total_tau_es_ * 0;
    }
  }
}

void ForcePositionFeedbackController::initialize_controller(FrankaRobot *robot) {
  model_ = robot->getModel();
}

void ForcePositionFeedbackController::get_next_step(const franka::RobotState &robot_state,
                                             TrajectoryGenerator *traj_generator) {
  
  auto fp_traj_generator = dynamic_cast<ForcePositionTrajectoryGenerator*>(traj_generator);
  if (fp_traj_generator == nullptr) {
    throw std::bad_cast();
  } 

  // Actual transform and force
  Eigen::Affine3d actual_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  std::array<double, 6> actual_force_array = robot_state.O_F_ext_hat_K;
  Eigen::Map<Eigen::VectorXd> actual_force(actual_force_array.data(), 6);
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());

  // Target transform and force
  Eigen::Affine3d desired_transform(Eigen::Matrix4d::Map(fp_traj_generator->get_desired_pose().data()));
  std::array<double, 6> desired_force_array = fp_traj_generator->get_desired_force();
  Eigen::Map<Eigen::VectorXd> desired_force(desired_force_array.data(), 6);

  // Compute errors
  Eigen::Vector3d translation_error = desired_transform.translation() - actual_transform.translation();
  Eigen::Quaterniond desired_quat(desired_transform.linear());
  Eigen::Quaterniond actual_quat(actual_transform.linear());
  Eigen::Quaterniond err_quat(desired_quat * actual_quat.inverse());
  Eigen::AngleAxisd err_angle_axis(err_quat);

  // Dynamics
  std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
  std::array<double, 42> jacobian_array = model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  // compute control
  xe_ << translation_error, err_angle_axis.axis() * err_angle_axis.angle();

  // b/c force is already a "delta", we can transform actual force directly to error frame
  // we ignore torque error for now - that remains in base frame
  fe_.head(3) << desired_force.head(3) - R_err * actual_force.head(3);
  fe_.tail(3) << desired_force.tail(3) - actual_force.tail(3);

  // transform translation error to error frame, ignoring rotation error
  xes_.head(3) << R_err.transpose() * S_.block(0, 0, 3, 3) * R_err * xe_.head(3);
  xes_.tail(3) << S_.block(3, 3, 3, 3) * xe_.tail(3);

  // transform force error back to base frame
  fes_.head(3) << R_err.transpose() * Sp_.block(0, 0, 3, 3) * fe_.head(3);
  fes_.tail(3) << Sp_.block(3, 3, 3, 3) * fe_.tail(3);

  if (actual_force.head(3).norm() > 2.) {
    printf("in contact\n");
  } else {
    printf("not in contact\n");
  }
  printf("fd: "); for (int i = 0; i < 3; i++) printf("%f, ", desired_force(i)); printf("\n");
  printf("fe: "); for (int i = 0; i < 3; i++) printf("%f, ", fe_(i)); printf("\n");
  printf("fes: "); for (int i = 0; i < 3; i++) printf("%f, ", fes_(i)); printf("\n");
  printf("\n");
  // printf("xe: "); for (int i = 0; i < 6; i++) printf("%f, ", xe_(i)); printf("\n");
  // printf("xes: "); for (int i = 0; i < 6; i++) printf("%f, ", xes_(i)); printf("\n");

  if (use_cartesian_gains_) {
    xad_ << jacobian * dq; 

    // compute positinoal dampign term in error frame
    xads_.head(3) << R_err.transpose() * S_.block(0, 0, 3, 3) * R_err * xad_.head(3);
    xads_.tail(3) << S_.block(3, 3, 3, 3) * xad_.tail(3);

    tau_x_ << jacobian.transpose() * (position_kps_cart_ * xes_ - position_kds_cart_ * xads_);
    tau_f_ << jacobian.transpose() * (force_kps_cart_ * fes_ + force_kis_cart_ * total_fes_);

    total_fes_ << total_fes_ + fes_;
  } else {
    q_es_ << jacobian.transpose() * xes_;
    tau_es_ << jacobian.transpose() * fes_;
    tau_x_ << position_kps_joint_ * q_es_ - position_kds_joint_ * dq;
    tau_f_ << force_kps_joint_ * tau_es_ + force_kis_joint_ * total_tau_es_;

    total_tau_es_ << total_tau_es_ + tau_es_;
  }

  tau_task_ << tau_x_ + tau_f_;
  tau_d_ << tau_task_ + coriolis;

  Eigen::VectorXd::Map(&tau_d_array_[0], 7) = tau_d_;
}

