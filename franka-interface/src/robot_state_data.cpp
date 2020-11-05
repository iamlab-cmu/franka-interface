#include "franka-interface/robot_state_data.h"

#include <iostream>

#include "franka-interface/run_loop.h"
#include "franka-interface/file_stream_logger.h"

//std::atomic<bool> RobotStateData::use_buffer_0{true};
//std::mutex RobotStateData::buffer_0_mutex_;
//std::mutex RobotStateData::buffer_1_mutex_;

template<int N>
void printListOfVectors(std::vector<std::array<double, N>> data, int print_last) {
  int print_counter = 0;
  for (auto it = data.rbegin(); it != data.rend(); it++, print_counter++) {
    std::array<double, N> temp = *it;
    for (size_t j = 0; j < temp.size(); j++) {
      std::cout << temp[j] << " ";
    }
    std::cout << "\n" << std::endl;
    if (++print_counter < print_last) {
      break;
    }
  }
}

void RobotStateData::setFileStreamLogger(FileStreamLogger *logger) {
  if (log_) {
    file_logger_ = logger;
    use_buffer_0 = true;

    file_logger_->initializeFile();
  }
}

void RobotStateData::updateFileStreamLogger(std::string new_filename) {
  if (log_) {
    file_logger_->updateFileName(new_filename);
    file_logger_->initializeFile();
  }
}

void RobotStateData::writeBufferData_0() {
    std::lock_guard<std::mutex> lock(buffer_0_mutex_);
    // std::cout << "Will save buffer 0\n";

    if (log_ && file_logger_ != nullptr) {
        if (log_skill_info_0_.size() > 0) {
          // below line doesn't actually return anything right now
          bool result = file_logger_->writeStringData(log_skill_info_0_);
          if (result) {
            // the line below inundates the logs
            // std::cout << "Success: Did write string data from buffer 0." << std::endl;
          } else {
            std::cout << "Fail: Did not write string data from buffer 0." << std::endl;
          }
        }

        bool result = file_logger_->writeData(log_time_since_skill_started_0_,
                                              log_pose_desired_0_,
                                              log_O_T_EE_0_,
                                              log_O_T_EE_d_0_,
                                              log_F_T_EE_0_,
                                              log_EE_T_K_0_,
                                              log_m_ee_0_,
                                              log_I_ee_0_,
                                              log_F_x_Cee_0_,
                                              log_m_load_0_,
                                              log_I_load_0_,
                                              log_F_x_Cload_0_,
                                              log_m_total_0_,
                                              log_I_total_0_,
                                              log_F_x_Ctotal_0_,
                                              log_elbow_0_,
                                              log_elbow_d_0_,
                                              log_elbow_c_0_,
                                              log_delbow_c_0_,
                                              log_ddelbow_c_0_,
                                              log_tau_J_0_,
                                              log_tau_J_d_0_,
                                              log_dtau_J_0_,
                                              log_q_0_,
                                              log_q_d_0_,
                                              log_dq_0_,
                                              log_dq_d_0_,
                                              log_ddq_d_0_,
                                              log_joint_contact_0_,
                                              log_cartesian_contact_0_,
                                              log_joint_collision_0_,
                                              log_cartesian_collision_0_,
                                              log_tau_ext_hat_filtered_0_,
                                              log_O_F_ext_hat_K_0_,
                                              log_K_F_ext_hat_K_0_,
                                              log_O_dP_EE_d_0_,
                                              log_O_T_EE_c_0_,
                                              log_O_dP_EE_c_0_,
                                              log_O_ddP_EE_c_0_,
                                              log_theta_0_,
                                              log_dtheta_0_,
                                              log_frames_0_,
                                              log_current_errors_0_,
                                              log_last_motion_errors_0_,
                                              log_control_command_success_rate_0_,
                                              log_robot_mode_0_,
                                              log_robot_time_0_,
                                              log_gripper_width_0_,
                                              log_gripper_max_width_0_,
                                              log_gripper_is_grasped_0_,
                                              log_gripper_temperature_0_,
                                              log_gripper_time_0_);
        if (result) {
            // std::cout << "Success: Did write data from buffer 0." << std::endl;
        } else {
            std::cout << "Fail: Did not write data from buffer 0." << std::endl;
        }
    }

    // For now just clear them
    log_skill_info_0_.clear();
    log_time_since_skill_started_0_.clear();

    log_pose_desired_0_.clear();
    log_O_T_EE_0_.clear();
    log_O_T_EE_d_0_.clear();
    log_F_T_EE_0_.clear();
    log_EE_T_K_0_.clear();
    log_m_ee_0_.clear();
    log_I_ee_0_.clear();
    log_F_x_Cee_0_.clear();
    log_m_load_0_.clear();
    log_I_load_0_.clear();
    log_F_x_Cload_0_.clear();
    log_m_total_0_.clear();
    log_I_total_0_.clear();
    log_F_x_Ctotal_0_.clear();
    log_elbow_0_.clear();
    log_elbow_d_0_.clear();
    log_elbow_c_0_.clear();
    log_delbow_c_0_.clear();
    log_ddelbow_c_0_.clear();
    log_tau_J_0_.clear();
    log_tau_J_d_0_.clear();
    log_dtau_J_0_.clear();
    log_q_0_.clear();
    log_q_d_0_.clear();
    log_dq_0_.clear();
    log_dq_d_0_.clear();
    log_ddq_d_0_.clear();
    log_joint_contact_0_.clear();
    log_cartesian_contact_0_.clear();
    log_joint_collision_0_.clear();
    log_cartesian_collision_0_.clear();
    log_tau_ext_hat_filtered_0_.clear();
    log_O_F_ext_hat_K_0_.clear();
    log_K_F_ext_hat_K_0_.clear();
    log_O_dP_EE_d_0_.clear();
    log_O_T_EE_c_0_.clear();
    log_O_dP_EE_c_0_.clear();
    log_O_ddP_EE_c_0_.clear();
    log_theta_0_.clear();
    log_dtheta_0_.clear();
    log_frames_0_.clear();
    log_current_errors_0_.clear();
    log_last_motion_errors_0_.clear();
    log_control_command_success_rate_0_.clear();
    log_robot_mode_0_.clear();
    log_robot_time_0_.clear();
    
    log_gripper_width_0_.clear();
    log_gripper_max_width_0_.clear();
    log_gripper_is_grasped_0_.clear();
    log_gripper_temperature_0_.clear();
    log_gripper_time_0_.clear();

    // std::cout << "Did save buffer 0\n";
};

void RobotStateData::writeBufferData_1() {
    std::lock_guard<std::mutex> lock(buffer_1_mutex_);
    // std::cout << "Will save buffer 1\n";

    if (log_ && file_logger_ != nullptr) {
        if (log_skill_info_1_.size() > 0) {
          // below line doesn't actually return anythin right now
          bool result = file_logger_->writeStringData(log_skill_info_1_);
          if (result) {
            // the line below inundates the logs
            // std::cout << "Success: Did write string data from buffer 0." << std::endl;
          } else {
            std::cout << "Fail: Did not write string data from buffer 0." << std::endl;
          }
        }
        bool result = file_logger_->writeData(log_time_since_skill_started_1_,
                                              log_pose_desired_1_,
                                              log_O_T_EE_1_,
                                              log_O_T_EE_d_1_,
                                              log_F_T_EE_1_,
                                              log_EE_T_K_1_,
                                              log_m_ee_1_,
                                              log_I_ee_1_,
                                              log_F_x_Cee_1_,
                                              log_m_load_1_,
                                              log_I_load_1_,
                                              log_F_x_Cload_1_,
                                              log_m_total_1_,
                                              log_I_total_1_,
                                              log_F_x_Ctotal_1_,
                                              log_elbow_1_,
                                              log_elbow_d_1_,
                                              log_elbow_c_1_,
                                              log_delbow_c_1_,
                                              log_ddelbow_c_1_,
                                              log_tau_J_1_,
                                              log_tau_J_d_1_,
                                              log_dtau_J_1_,
                                              log_q_1_,
                                              log_q_d_1_,
                                              log_dq_1_,
                                              log_dq_d_1_,
                                              log_ddq_d_1_,
                                              log_joint_contact_1_,
                                              log_cartesian_contact_1_,
                                              log_joint_collision_1_,
                                              log_cartesian_collision_1_,
                                              log_tau_ext_hat_filtered_1_,
                                              log_O_F_ext_hat_K_1_,
                                              log_K_F_ext_hat_K_1_,
                                              log_O_dP_EE_d_1_,
                                              log_O_T_EE_c_1_,
                                              log_O_dP_EE_c_1_,
                                              log_O_ddP_EE_c_1_,
                                              log_theta_1_,
                                              log_dtheta_1_,
                                              log_frames_1_,
                                              log_current_errors_1_,
                                              log_last_motion_errors_1_,
                                              log_control_command_success_rate_1_,
                                              log_robot_mode_1_,
                                              log_robot_time_1_,
                                              log_gripper_width_1_,
                                              log_gripper_max_width_1_,
                                              log_gripper_is_grasped_1_,
                                              log_gripper_temperature_1_,
                                              log_gripper_time_1_);
        if (result) {
          // std::cout << "Success: Did write data from buffer 1." << std::endl;
        } else {
          std::cout << "Fail: Did not write data from buffer 1." << std::endl;
        }
    }

    log_skill_info_1_.clear();
    log_time_since_skill_started_1_.clear();

    log_pose_desired_1_.clear();
    log_O_T_EE_1_.clear();
    log_O_T_EE_d_1_.clear();
    log_F_T_EE_1_.clear();
    log_EE_T_K_1_.clear();
    log_m_ee_1_.clear();
    log_I_ee_1_.clear();
    log_F_x_Cee_1_.clear();
    log_m_load_1_.clear();
    log_I_load_1_.clear();
    log_F_x_Cload_1_.clear();
    log_m_total_1_.clear();
    log_I_total_1_.clear();
    log_F_x_Ctotal_1_.clear();
    log_elbow_1_.clear();
    log_elbow_d_1_.clear();
    log_elbow_c_1_.clear();
    log_delbow_c_1_.clear();
    log_ddelbow_c_1_.clear();
    log_tau_J_1_.clear();
    log_tau_J_d_1_.clear();
    log_dtau_J_1_.clear();
    log_q_1_.clear();
    log_q_d_1_.clear();
    log_dq_1_.clear();
    log_dq_d_1_.clear();
    log_ddq_d_1_.clear();
    log_joint_contact_1_.clear();
    log_cartesian_contact_1_.clear();
    log_joint_collision_1_.clear();
    log_cartesian_collision_1_.clear();
    log_tau_ext_hat_filtered_1_.clear();
    log_O_F_ext_hat_K_1_.clear();
    log_K_F_ext_hat_K_1_.clear();
    log_O_dP_EE_d_1_.clear();
    log_O_T_EE_c_1_.clear();
    log_O_dP_EE_c_1_.clear();
    log_O_ddP_EE_c_1_.clear();
    log_theta_1_.clear();
    log_dtheta_1_.clear();
    log_frames_1_.clear();
    log_current_errors_1_.clear();
    log_last_motion_errors_1_.clear();
    log_control_command_success_rate_1_.clear();
    log_robot_mode_1_.clear();
    log_robot_time_1_.clear();

    log_gripper_width_1_.clear();
    log_gripper_max_width_1_.clear();
    log_gripper_is_grasped_1_.clear();
    log_gripper_temperature_1_.clear();
    log_gripper_time_1_.clear();

    // std::cout << "Did save buffer 1\n";
};

void RobotStateData::clearAllBuffers() {
  std::lock_guard<std::mutex> lock_0(buffer_0_mutex_);
  std::lock_guard<std::mutex> lock_1(buffer_1_mutex_);
  
  log_skill_info_0_.clear();
  log_time_since_skill_started_0_.clear();

  log_pose_desired_0_.clear();
  log_O_T_EE_0_.clear();
  log_O_T_EE_d_0_.clear();
  log_F_T_EE_0_.clear();
  log_EE_T_K_0_.clear();
  log_m_ee_0_.clear();
  log_I_ee_0_.clear();
  log_F_x_Cee_0_.clear();
  log_m_load_0_.clear();
  log_I_load_0_.clear();
  log_F_x_Cload_0_.clear();
  log_m_total_0_.clear();
  log_I_total_0_.clear();
  log_F_x_Ctotal_0_.clear();
  log_elbow_0_.clear();
  log_elbow_d_0_.clear();
  log_elbow_c_0_.clear();
  log_delbow_c_0_.clear();
  log_ddelbow_c_0_.clear();
  log_tau_J_0_.clear();
  log_tau_J_d_0_.clear();
  log_dtau_J_0_.clear();
  log_q_0_.clear();
  log_q_d_0_.clear();
  log_dq_0_.clear();
  log_dq_d_0_.clear();
  log_ddq_d_0_.clear();
  log_joint_contact_0_.clear();
  log_cartesian_contact_0_.clear();
  log_joint_collision_0_.clear();
  log_cartesian_collision_0_.clear();
  log_tau_ext_hat_filtered_0_.clear();
  log_O_F_ext_hat_K_0_.clear();
  log_K_F_ext_hat_K_0_.clear();
  log_O_dP_EE_d_0_.clear();
  log_O_T_EE_c_0_.clear();
  log_O_dP_EE_c_0_.clear();
  log_O_ddP_EE_c_0_.clear();
  log_theta_0_.clear();
  log_dtheta_0_.clear();
  log_frames_0_.clear();
  log_current_errors_0_.clear();
  log_last_motion_errors_0_.clear();
  log_control_command_success_rate_0_.clear();
  log_robot_mode_0_.clear();
  log_robot_time_0_.clear();

  log_gripper_width_0_.clear();
  log_gripper_max_width_0_.clear();
  log_gripper_is_grasped_0_.clear();
  log_gripper_temperature_0_.clear();
  log_gripper_time_0_.clear();

  log_skill_info_1_.clear();
  log_time_since_skill_started_1_.clear();

  log_pose_desired_1_.clear();
  log_O_T_EE_1_.clear();
  log_O_T_EE_d_1_.clear();
  log_F_T_EE_1_.clear();
  log_EE_T_K_1_.clear();
  log_m_ee_1_.clear();
  log_I_ee_1_.clear();
  log_F_x_Cee_1_.clear();
  log_m_load_1_.clear();
  log_I_load_1_.clear();
  log_F_x_Cload_1_.clear();
  log_m_total_1_.clear();
  log_I_total_1_.clear();
  log_F_x_Ctotal_1_.clear();
  log_elbow_1_.clear();
  log_elbow_d_1_.clear();
  log_elbow_c_1_.clear();
  log_delbow_c_1_.clear();
  log_ddelbow_c_1_.clear();
  log_tau_J_1_.clear();
  log_tau_J_d_1_.clear();
  log_dtau_J_1_.clear();
  log_q_1_.clear();
  log_q_d_1_.clear();
  log_dq_1_.clear();
  log_dq_d_1_.clear();
  log_ddq_d_1_.clear();
  log_joint_contact_1_.clear();
  log_cartesian_contact_1_.clear();
  log_joint_collision_1_.clear();
  log_cartesian_collision_1_.clear();
  log_tau_ext_hat_filtered_1_.clear();
  log_O_F_ext_hat_K_1_.clear();
  log_K_F_ext_hat_K_1_.clear();
  log_O_dP_EE_d_1_.clear();
  log_O_T_EE_c_1_.clear();
  log_O_dP_EE_c_1_.clear();
  log_O_ddP_EE_c_1_.clear();
  log_theta_1_.clear();
  log_dtheta_1_.clear();
  log_frames_1_.clear();
  log_current_errors_1_.clear();
  log_last_motion_errors_1_.clear();
  log_control_command_success_rate_1_.clear();
  log_robot_mode_1_.clear();
  log_robot_time_1_.clear();

  log_gripper_width_1_.clear();
  log_gripper_max_width_1_.clear();
  log_gripper_is_grasped_1_.clear();
  log_gripper_temperature_1_.clear();
  log_gripper_time_1_.clear();
  
  current_gripper_width_ = -1.0;
  current_gripper_max_width_ = -1.0;
  current_gripper_is_grasped_ = false;
  current_gripper_temperature_ = 0;
  current_gripper_time_ = -1.0;
}

void RobotStateData::startFileLoggerThread() {
    file_logger_thread_ = std::thread([&]() {
      // Sleep to achieve the desired print rate.
      while (true) {
          std::this_thread::sleep_for(
              std::chrono::milliseconds(static_cast<int>((1.0 / log_rate_ * 1000.0))));

          if (!run_loop::run_loop_ok_) {
            continue;
          }
          
          // Try to lock data to avoid read write collisions.
          bool did_write_to_buffer_0 = false;

          // Control loop thread should now switch between buffers.
          if (use_buffer_0) {
            use_buffer_0 = false;
            did_write_to_buffer_0 = true;
          } else {
            use_buffer_0 = true;
          }
          if (did_write_to_buffer_0) {
            writeBufferData_0();
          } else {
            writeBufferData_1();
          }
      }
    });
}

void RobotStateData::writeCurrentBufferData() {
  if (use_buffer_0) {
    writeBufferData_0();
  } else {
    writeBufferData_1();
  }
}

// TODO(jacky): switch to printing buffer 0 or 1, b/c global data buffers are not actually populated.
void RobotStateData::printData(int print_count) {
  if (use_buffer_0) {
    std::cout << "===== Robots state ======\n";
    printListOfVectors<16>(log_O_T_EE_0_, print_count);

    std::cout << "===== Desired Pose ======\n";
    printListOfVectors<16>(log_O_T_EE_d_0_, print_count);

    std::cout << "===== Measured link-side joint torque sensor signals ======\n";
    printListOfVectors<7>(log_tau_J_0_, print_count);

    std::cout << "===== Measured joint velocity ======\n";
    printListOfVectors<7>(log_dq_0_, print_count);

    std::cout << "===== Measured external force and torque ======\n";
    printListOfVectors<6>(log_O_F_ext_hat_K_0_, print_count);
  } else {
    std::cout << "===== Robots state ======\n";
    printListOfVectors<16>(log_O_T_EE_1_, print_count);

    std::cout << "===== Desired Pose ======\n";
    printListOfVectors<16>(log_O_T_EE_d_1_, print_count);

    std::cout << "===== Measured link-side joint torque sensor signals ======\n";
    printListOfVectors<7>(log_tau_J_1_, print_count);

    std::cout << "===== Measured joint velocity ======\n";
    printListOfVectors<7>(log_dq_1_, print_count);

    std::cout << "===== Measured external force and torque ======\n";
    printListOfVectors<6>(log_O_F_ext_hat_K_1_, print_count);
  }
}

void RobotStateData::log_robot_state(std::array<double, 16> &desired_pose, franka::RobotState robot_state, franka::Model *robot_model, double time_since_skill_started) {
  current_robot_state_ = robot_state;

  current_pose_desired_ = desired_pose;

  std::array<bool, 37> current_errors;
  current_errors[0] = robot_state.current_errors.joint_position_limits_violation;
  current_errors[1] = robot_state.current_errors.cartesian_position_limits_violation;
  current_errors[2] = robot_state.current_errors.self_collision_avoidance_violation;
  current_errors[3] = robot_state.current_errors.joint_velocity_violation;
  current_errors[4] = robot_state.current_errors.cartesian_velocity_violation;
  current_errors[5] = robot_state.current_errors.force_control_safety_violation;
  current_errors[6] = robot_state.current_errors.joint_reflex;
  current_errors[7] = robot_state.current_errors.cartesian_reflex;
  current_errors[8] = robot_state.current_errors.max_goal_pose_deviation_violation;
  current_errors[9] = robot_state.current_errors.max_path_pose_deviation_violation;
  current_errors[10] = robot_state.current_errors.cartesian_velocity_profile_safety_violation;
  current_errors[11] = robot_state.current_errors.joint_position_motion_generator_start_pose_invalid;
  current_errors[12] = robot_state.current_errors.joint_motion_generator_position_limits_violation;
  current_errors[13] = robot_state.current_errors.joint_motion_generator_velocity_limits_violation;
  current_errors[14] = robot_state.current_errors.joint_motion_generator_velocity_discontinuity;
  current_errors[15] = robot_state.current_errors.joint_motion_generator_acceleration_discontinuity;
  current_errors[16] = robot_state.current_errors.cartesian_position_motion_generator_start_pose_invalid;
  current_errors[17] = robot_state.current_errors.cartesian_motion_generator_elbow_limit_violation;
  current_errors[18] = robot_state.current_errors.cartesian_motion_generator_velocity_limits_violation;
  current_errors[19] = robot_state.current_errors.cartesian_motion_generator_velocity_discontinuity;
  current_errors[20] = robot_state.current_errors.cartesian_motion_generator_acceleration_discontinuity;
  current_errors[21] = robot_state.current_errors.cartesian_motion_generator_elbow_sign_inconsistent;
  current_errors[22] = robot_state.current_errors.cartesian_motion_generator_start_elbow_invalid;
  current_errors[23] = robot_state.current_errors.cartesian_motion_generator_joint_position_limits_violation;
  current_errors[24] = robot_state.current_errors.cartesian_motion_generator_joint_velocity_limits_violation;
  current_errors[25] = robot_state.current_errors.cartesian_motion_generator_joint_velocity_discontinuity;
  current_errors[26] = robot_state.current_errors.cartesian_motion_generator_joint_acceleration_discontinuity;
  current_errors[27] = robot_state.current_errors.cartesian_position_motion_generator_invalid_frame;
  current_errors[28] = robot_state.current_errors.force_controller_desired_force_tolerance_violation;
  current_errors[29] = robot_state.current_errors.controller_torque_discontinuity;
  current_errors[30] = robot_state.current_errors.start_elbow_sign_inconsistent;
  current_errors[31] = robot_state.current_errors.communication_constraints_violation;
  current_errors[32] = robot_state.current_errors.power_limit_violation;
  current_errors[33] = robot_state.current_errors.joint_p2p_insufficient_torque_for_planning;
  current_errors[34] = robot_state.current_errors.tau_j_range_violation;
  current_errors[35] = robot_state.current_errors.instability_detected;
  current_errors[36] = robot_state.current_errors.joint_move_in_wrong_direction;

  std::array<bool, 37> last_motion_errors;
  last_motion_errors[0] = robot_state.last_motion_errors.joint_position_limits_violation;
  last_motion_errors[1] = robot_state.last_motion_errors.cartesian_position_limits_violation;
  last_motion_errors[2] = robot_state.last_motion_errors.self_collision_avoidance_violation;
  last_motion_errors[3] = robot_state.last_motion_errors.joint_velocity_violation;
  last_motion_errors[4] = robot_state.last_motion_errors.cartesian_velocity_violation;
  last_motion_errors[5] = robot_state.last_motion_errors.force_control_safety_violation;
  last_motion_errors[6] = robot_state.last_motion_errors.joint_reflex;
  last_motion_errors[7] = robot_state.last_motion_errors.cartesian_reflex;
  last_motion_errors[8] = robot_state.last_motion_errors.max_goal_pose_deviation_violation;
  last_motion_errors[9] = robot_state.last_motion_errors.max_path_pose_deviation_violation;
  last_motion_errors[10] = robot_state.last_motion_errors.cartesian_velocity_profile_safety_violation;
  last_motion_errors[11] = robot_state.last_motion_errors.joint_position_motion_generator_start_pose_invalid;
  last_motion_errors[12] = robot_state.last_motion_errors.joint_motion_generator_position_limits_violation;
  last_motion_errors[13] = robot_state.last_motion_errors.joint_motion_generator_velocity_limits_violation;
  last_motion_errors[14] = robot_state.last_motion_errors.joint_motion_generator_velocity_discontinuity;
  last_motion_errors[15] = robot_state.last_motion_errors.joint_motion_generator_acceleration_discontinuity;
  last_motion_errors[16] = robot_state.last_motion_errors.cartesian_position_motion_generator_start_pose_invalid;
  last_motion_errors[17] = robot_state.last_motion_errors.cartesian_motion_generator_elbow_limit_violation;
  last_motion_errors[18] = robot_state.last_motion_errors.cartesian_motion_generator_velocity_limits_violation;
  last_motion_errors[19] = robot_state.last_motion_errors.cartesian_motion_generator_velocity_discontinuity;
  last_motion_errors[20] = robot_state.last_motion_errors.cartesian_motion_generator_acceleration_discontinuity;
  last_motion_errors[21] = robot_state.last_motion_errors.cartesian_motion_generator_elbow_sign_inconsistent;
  last_motion_errors[22] = robot_state.last_motion_errors.cartesian_motion_generator_start_elbow_invalid;
  last_motion_errors[23] = robot_state.last_motion_errors.cartesian_motion_generator_joint_position_limits_violation;
  last_motion_errors[24] = robot_state.last_motion_errors.cartesian_motion_generator_joint_velocity_limits_violation;
  last_motion_errors[25] = robot_state.last_motion_errors.cartesian_motion_generator_joint_velocity_discontinuity;
  last_motion_errors[26] = robot_state.last_motion_errors.cartesian_motion_generator_joint_acceleration_discontinuity;
  last_motion_errors[27] = robot_state.last_motion_errors.cartesian_position_motion_generator_invalid_frame;
  last_motion_errors[28] = robot_state.last_motion_errors.force_controller_desired_force_tolerance_violation;
  last_motion_errors[29] = robot_state.last_motion_errors.controller_torque_discontinuity;
  last_motion_errors[30] = robot_state.last_motion_errors.start_elbow_sign_inconsistent;
  last_motion_errors[31] = robot_state.last_motion_errors.communication_constraints_violation;
  last_motion_errors[32] = robot_state.last_motion_errors.power_limit_violation;
  last_motion_errors[33] = robot_state.last_motion_errors.joint_p2p_insufficient_torque_for_planning;
  last_motion_errors[34] = robot_state.last_motion_errors.tau_j_range_violation;
  last_motion_errors[35] = robot_state.last_motion_errors.instability_detected;
  last_motion_errors[36] = robot_state.last_motion_errors.joint_move_in_wrong_direction;

  int n_frame = 0;
  for (franka::Frame frame = franka::Frame::kJoint1; frame <= franka::Frame::kEndEffector; frame++) {
      auto pose = robot_model->pose(frame, robot_state);
      for (int i = 0; i < 16; i++)
      {
        current_robot_frames_[n_frame * 16 + i] = pose[i];
      }
      n_frame++;
  }

  if (use_buffer_0) {
    if (buffer_0_mutex_.try_lock()) {
      log_time_since_skill_started_0_.push_back(time_since_skill_started);

      log_pose_desired_0_.push_back(desired_pose);
      log_O_T_EE_0_.push_back(robot_state.O_T_EE);
      log_O_T_EE_d_0_.push_back(robot_state.O_T_EE_d);
      log_F_T_EE_0_.push_back(robot_state.F_T_EE);
      log_EE_T_K_0_.push_back(robot_state.EE_T_K);
      log_m_ee_0_.push_back(robot_state.m_ee);
      log_I_ee_0_.push_back(robot_state.I_ee);
      log_F_x_Cee_0_.push_back(robot_state.F_x_Cee);
      log_m_load_0_.push_back(robot_state.m_load);
      log_I_load_0_.push_back(robot_state.I_load);
      log_F_x_Cload_0_.push_back(robot_state.F_x_Cload);
      log_m_total_0_.push_back(robot_state.m_total);
      log_I_total_0_.push_back(robot_state.I_total);
      log_F_x_Ctotal_0_.push_back(robot_state.F_x_Ctotal);
      log_elbow_0_.push_back(robot_state.elbow);
      log_elbow_d_0_.push_back(robot_state.elbow_d);
      log_elbow_c_0_.push_back(robot_state.elbow_c);
      log_delbow_c_0_.push_back(robot_state.delbow_c);
      log_ddelbow_c_0_.push_back(robot_state.ddelbow_c);
      log_tau_J_0_.push_back(robot_state.tau_J);
      log_tau_J_d_0_.push_back(robot_state.tau_J_d);
      log_dtau_J_0_.push_back(robot_state.dtau_J);
      log_q_0_.push_back(robot_state.q);
      log_q_d_0_.push_back(robot_state.q_d);
      log_dq_0_.push_back(robot_state.dq);
      log_dq_d_0_.push_back(robot_state.dq_d);
      log_ddq_d_0_.push_back(robot_state.ddq_d);
      log_joint_contact_0_.push_back(robot_state.joint_contact);
      log_cartesian_contact_0_.push_back(robot_state.cartesian_contact);
      log_joint_collision_0_.push_back(robot_state.joint_collision);
      log_cartesian_collision_0_.push_back(robot_state.cartesian_collision);
      log_tau_ext_hat_filtered_0_.push_back(robot_state.tau_ext_hat_filtered);
      log_O_F_ext_hat_K_0_.push_back(robot_state.O_F_ext_hat_K);
      log_K_F_ext_hat_K_0_.push_back(robot_state.K_F_ext_hat_K);
      log_O_dP_EE_d_0_.push_back(robot_state.O_dP_EE_d);
      log_O_T_EE_c_0_.push_back(robot_state.O_T_EE_c);
      log_O_dP_EE_c_0_.push_back(robot_state.O_dP_EE_c);
      log_O_ddP_EE_c_0_.push_back(robot_state.O_ddP_EE_c);
      log_theta_0_.push_back(robot_state.theta);
      log_dtheta_0_.push_back(robot_state.dtheta);
      log_frames_0_.push_back(current_robot_frames_);
      log_current_errors_0_.push_back(current_errors);
      log_last_motion_errors_0_.push_back(last_motion_errors);
      log_control_command_success_rate_0_.push_back(robot_state.control_command_success_rate);
      log_robot_mode_0_.push_back(static_cast<uint8_t>(robot_state.robot_mode));
      log_robot_time_0_.push_back(robot_state.time.toSec());

      log_gripper_width_0_.push_back(current_gripper_width_);
      log_gripper_max_width_0_.push_back(current_gripper_max_width_);
      log_gripper_is_grasped_0_.push_back(current_gripper_is_grasped_);
      log_gripper_temperature_0_.push_back(current_gripper_temperature_);
      log_gripper_time_0_.push_back(current_gripper_time_);

      buffer_0_mutex_.unlock();
    }
  } else {
    if (buffer_1_mutex_.try_lock()) {
      log_time_since_skill_started_1_.push_back(time_since_skill_started);

      log_pose_desired_1_.push_back(desired_pose);
      log_O_T_EE_1_.push_back(robot_state.O_T_EE);
      log_O_T_EE_d_1_.push_back(robot_state.O_T_EE_d);
      log_F_T_EE_1_.push_back(robot_state.F_T_EE);
      log_EE_T_K_1_.push_back(robot_state.EE_T_K);
      log_m_ee_1_.push_back(robot_state.m_ee);
      log_I_ee_1_.push_back(robot_state.I_ee);
      log_F_x_Cee_1_.push_back(robot_state.F_x_Cee);
      log_m_load_1_.push_back(robot_state.m_load);
      log_I_load_1_.push_back(robot_state.I_load);
      log_F_x_Cload_1_.push_back(robot_state.F_x_Cload);
      log_m_total_1_.push_back(robot_state.m_total);
      log_I_total_1_.push_back(robot_state.I_total);
      log_F_x_Ctotal_1_.push_back(robot_state.F_x_Ctotal);
      log_elbow_1_.push_back(robot_state.elbow);
      log_elbow_d_1_.push_back(robot_state.elbow_d);
      log_elbow_c_1_.push_back(robot_state.elbow_c);
      log_delbow_c_1_.push_back(robot_state.delbow_c);
      log_ddelbow_c_1_.push_back(robot_state.ddelbow_c);
      log_tau_J_1_.push_back(robot_state.tau_J);
      log_tau_J_d_1_.push_back(robot_state.tau_J_d);
      log_dtau_J_1_.push_back(robot_state.dtau_J);
      log_q_1_.push_back(robot_state.q);
      log_q_d_1_.push_back(robot_state.q_d);
      log_dq_1_.push_back(robot_state.dq);
      log_dq_d_1_.push_back(robot_state.dq_d);
      log_ddq_d_1_.push_back(robot_state.ddq_d);
      log_joint_contact_1_.push_back(robot_state.joint_contact);
      log_cartesian_contact_1_.push_back(robot_state.cartesian_contact);
      log_joint_collision_1_.push_back(robot_state.joint_collision);
      log_cartesian_collision_1_.push_back(robot_state.cartesian_collision);
      log_tau_ext_hat_filtered_1_.push_back(robot_state.tau_ext_hat_filtered);
      log_O_F_ext_hat_K_1_.push_back(robot_state.O_F_ext_hat_K);
      log_K_F_ext_hat_K_1_.push_back(robot_state.K_F_ext_hat_K);
      log_O_dP_EE_d_1_.push_back(robot_state.O_dP_EE_d);
      log_O_T_EE_c_1_.push_back(robot_state.O_T_EE_c);
      log_O_dP_EE_c_1_.push_back(robot_state.O_dP_EE_c);
      log_O_ddP_EE_c_1_.push_back(robot_state.O_ddP_EE_c);
      log_theta_1_.push_back(robot_state.theta);
      log_dtheta_1_.push_back(robot_state.dtheta);
      log_frames_1_.push_back(current_robot_frames_);
      log_current_errors_1_.push_back(current_errors);
      log_last_motion_errors_1_.push_back(last_motion_errors);
      log_control_command_success_rate_1_.push_back(robot_state.control_command_success_rate);
      log_robot_mode_1_.push_back(static_cast<uint8_t>(robot_state.robot_mode));
      log_robot_time_1_.push_back(robot_state.time.toSec());
      
      log_gripper_width_1_.push_back(current_gripper_width_);
      log_gripper_max_width_1_.push_back(current_gripper_max_width_);
      log_gripper_is_grasped_1_.push_back(current_gripper_is_grasped_);
      log_gripper_temperature_1_.push_back(current_gripper_temperature_);
      log_gripper_time_1_.push_back(current_gripper_time_);

      buffer_1_mutex_.unlock();
    }
  }
}


void RobotStateData::update_current_gripper_state(franka::GripperState gripper_state) {
  current_gripper_state_ = gripper_state;

  current_gripper_width_ = gripper_state.width;
  current_gripper_max_width_ = gripper_state.max_width;
  current_gripper_is_grasped_ = gripper_state.is_grasped;
  current_gripper_temperature_ = gripper_state.temperature;
  current_gripper_time_ = gripper_state.time.toSec();
}

void RobotStateData::log_skill_info(std::string skill_info) {
  if (use_buffer_0) {
    if (buffer_0_mutex_.try_lock()) {
      log_skill_info_0_.push_back(skill_info);
      buffer_0_mutex_.unlock();
    }
  } else {
    if (buffer_1_mutex_.try_lock()) {
      log_skill_info_1_.push_back(skill_info);
      buffer_1_mutex_.unlock();
    }
  }
}
