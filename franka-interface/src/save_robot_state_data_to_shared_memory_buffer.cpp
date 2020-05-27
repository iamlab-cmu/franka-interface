#include "franka-interface/save_robot_state_data_to_shared_memory_buffer.h"

void save_current_robot_state_data_to_shared_memory_buffer(RunLoopSharedMemoryHandler* shared_memory_handler,
                                                           RobotStateData* robot_state_data) {
  if (shared_memory_handler->getCurrentRobotStateBufferMutex()->try_lock()) {
    SharedBufferTypePtr current_robot_state_buffer = shared_memory_handler->getCurrentRobotStateBuffer();

    RobotStateMessage robot_state_msg;

    for (int i = 0; i < 16; i++) {
        robot_state_msg.add_pose_desired(robot_state_data->current_pose_desired_[i]);
        robot_state_msg.add_o_t_ee(robot_state_data->current_robot_state_.O_T_EE[i]);
        robot_state_msg.add_o_t_ee_d(robot_state_data->current_robot_state_.O_T_EE_d[i]);
        robot_state_msg.add_o_t_ee_c(robot_state_data->current_robot_state_.O_T_EE_c[i]);
        robot_state_msg.add_f_t_ee(robot_state_data->current_robot_state_.F_T_EE[i]);
        robot_state_msg.add_ee_t_k(robot_state_data->current_robot_state_.EE_T_K[i]);
    }
    
    robot_state_msg.set_m_ee(robot_state_data->current_robot_state_.m_ee);
    robot_state_msg.set_m_load(robot_state_data->current_robot_state_.m_load);
    robot_state_msg.set_m_total(robot_state_data->current_robot_state_.m_total);

    for (int i = 0; i < 9; i++) {
        robot_state_msg.add_i_ee(robot_state_data->current_robot_state_.I_ee[i]);
        robot_state_msg.add_i_load(robot_state_data->current_robot_state_.I_load[i]);
        robot_state_msg.add_i_total(robot_state_data->current_robot_state_.I_total[i]);
    }

    for (int i = 0; i < 3; i++) {
        robot_state_msg.add_f_x_cee(robot_state_data->current_robot_state_.F_x_Cee[i]);
        robot_state_msg.add_f_x_cload(robot_state_data->current_robot_state_.F_x_Cload[i]);
        robot_state_msg.add_f_x_ctotal(robot_state_data->current_robot_state_.F_x_Ctotal[i]);
    }

    for (int i = 0; i < 2; i++) {
        robot_state_msg.add_elbow(robot_state_data->current_robot_state_.elbow[i]);
        robot_state_msg.add_elbow_d(robot_state_data->current_robot_state_.elbow_d[i]);
        robot_state_msg.add_elbow_c(robot_state_data->current_robot_state_.elbow_c[i]);
        robot_state_msg.add_delbow_c(robot_state_data->current_robot_state_.delbow_c[i]);
        robot_state_msg.add_ddelbow_c(robot_state_data->current_robot_state_.ddelbow_c[i]);
    }    

    for (int i = 0; i < 7; i++) {
        robot_state_msg.add_tau_j(robot_state_data->current_robot_state_.tau_J[i]);
        robot_state_msg.add_tau_j_d(robot_state_data->current_robot_state_.tau_J_d[i]);
        robot_state_msg.add_dtau_j(robot_state_data->current_robot_state_.dtau_J[i]);
        robot_state_msg.add_q(robot_state_data->current_robot_state_.q[i]);
        robot_state_msg.add_q_d(robot_state_data->current_robot_state_.q_d[i]);
        robot_state_msg.add_dq(robot_state_data->current_robot_state_.dq[i]);
        robot_state_msg.add_dq_d(robot_state_data->current_robot_state_.dq_d[i]);
        robot_state_msg.add_ddq_d(robot_state_data->current_robot_state_.ddq_d[i]);
        robot_state_msg.add_joint_contact(robot_state_data->current_robot_state_.joint_contact[i]);
        robot_state_msg.add_joint_collision(robot_state_data->current_robot_state_.joint_collision[i]);
        robot_state_msg.add_tau_ext_hat_filtered(robot_state_data->current_robot_state_.tau_ext_hat_filtered[i]);
        robot_state_msg.add_theta(robot_state_data->current_robot_state_.theta[i]);
        robot_state_msg.add_dtheta(robot_state_data->current_robot_state_.dtheta[i]);
    }

    for (int i = 0; i < 6; i++) {
        robot_state_msg.add_cartesian_contact(robot_state_data->current_robot_state_.cartesian_contact[i]);
        robot_state_msg.add_cartesian_collision(robot_state_data->current_robot_state_.cartesian_collision[i]);
        robot_state_msg.add_o_f_ext_hat_k(robot_state_data->current_robot_state_.O_F_ext_hat_K[i]);
        robot_state_msg.add_k_f_ext_hat_k(robot_state_data->current_robot_state_.K_F_ext_hat_K[i]);
        robot_state_msg.add_o_dp_ee_d(robot_state_data->current_robot_state_.O_dP_EE_d[i]);
        robot_state_msg.add_o_dp_ee_c(robot_state_data->current_robot_state_.O_dP_EE_c[i]);
        robot_state_msg.add_o_ddp_ee_c(robot_state_data->current_robot_state_.O_ddP_EE_c[i]);
    }

    for (int i = 0; i < 144; i++) {
        robot_state_msg.add_robot_frames(static_cast<float>(robot_state_data->current_robot_frames_[i]));
    }

    robot_state_msg.set_current_errors_joint_position_limits_violation(robot_state_data->current_robot_state_.current_errors.joint_position_limits_violation);
    robot_state_msg.set_current_errors_cartesian_position_limits_violation(robot_state_data->current_robot_state_.current_errors.cartesian_position_limits_violation);
    robot_state_msg.set_current_errors_self_collision_avoidance_violation(robot_state_data->current_robot_state_.current_errors.self_collision_avoidance_violation);
    robot_state_msg.set_current_errors_joint_velocity_violation(robot_state_data->current_robot_state_.current_errors.joint_velocity_violation);
    robot_state_msg.set_current_errors_cartesian_velocity_violation(robot_state_data->current_robot_state_.current_errors.cartesian_velocity_violation);
    robot_state_msg.set_current_errors_force_control_safety_violation(robot_state_data->current_robot_state_.current_errors.force_control_safety_violation);
    robot_state_msg.set_current_errors_joint_reflex(robot_state_data->current_robot_state_.current_errors.joint_reflex);
    robot_state_msg.set_current_errors_cartesian_reflex(robot_state_data->current_robot_state_.current_errors.cartesian_reflex);
    robot_state_msg.set_current_errors_max_goal_pose_deviation_violation(robot_state_data->current_robot_state_.current_errors.max_goal_pose_deviation_violation);
    robot_state_msg.set_current_errors_max_path_pose_deviation_violation(robot_state_data->current_robot_state_.current_errors.max_path_pose_deviation_violation);
    robot_state_msg.set_current_errors_cartesian_velocity_profile_safety_violation(robot_state_data->current_robot_state_.current_errors.cartesian_velocity_profile_safety_violation);
    robot_state_msg.set_current_errors_joint_position_motion_generator_start_pose_invalid(robot_state_data->current_robot_state_.current_errors.joint_position_motion_generator_start_pose_invalid);
    robot_state_msg.set_current_errors_joint_motion_generator_position_limits_violation(robot_state_data->current_robot_state_.current_errors.joint_motion_generator_position_limits_violation);
    robot_state_msg.set_current_errors_joint_motion_generator_velocity_limits_violation(robot_state_data->current_robot_state_.current_errors.joint_motion_generator_velocity_limits_violation);
    robot_state_msg.set_current_errors_joint_motion_generator_velocity_discontinuity(robot_state_data->current_robot_state_.current_errors.joint_motion_generator_velocity_discontinuity);
    robot_state_msg.set_current_errors_joint_motion_generator_acceleration_discontinuity(robot_state_data->current_robot_state_.current_errors.joint_motion_generator_acceleration_discontinuity);
    robot_state_msg.set_current_errors_cartesian_position_motion_generator_start_pose_invalid(robot_state_data->current_robot_state_.current_errors.cartesian_position_motion_generator_start_pose_invalid);
    robot_state_msg.set_current_errors_cartesian_motion_generator_elbow_limit_violation(robot_state_data->current_robot_state_.current_errors.cartesian_motion_generator_elbow_limit_violation);
    robot_state_msg.set_current_errors_cartesian_motion_generator_velocity_limits_violation(robot_state_data->current_robot_state_.current_errors.cartesian_motion_generator_velocity_limits_violation);
    robot_state_msg.set_current_errors_cartesian_motion_generator_velocity_discontinuity(robot_state_data->current_robot_state_.current_errors.cartesian_motion_generator_velocity_discontinuity);
    robot_state_msg.set_current_errors_cartesian_motion_generator_acceleration_discontinuity(robot_state_data->current_robot_state_.current_errors.cartesian_motion_generator_acceleration_discontinuity);
    robot_state_msg.set_current_errors_cartesian_motion_generator_elbow_sign_inconsistent(robot_state_data->current_robot_state_.current_errors.cartesian_motion_generator_elbow_sign_inconsistent);
    robot_state_msg.set_current_errors_cartesian_motion_generator_start_elbow_invalid(robot_state_data->current_robot_state_.current_errors.cartesian_motion_generator_start_elbow_invalid);
    robot_state_msg.set_current_errors_cartesian_motion_generator_joint_position_limits_violation(robot_state_data->current_robot_state_.current_errors.cartesian_motion_generator_joint_position_limits_violation);
    robot_state_msg.set_current_errors_cartesian_motion_generator_joint_velocity_limits_violation(robot_state_data->current_robot_state_.current_errors.cartesian_motion_generator_joint_velocity_limits_violation);
    robot_state_msg.set_current_errors_cartesian_motion_generator_joint_velocity_discontinuity(robot_state_data->current_robot_state_.current_errors.cartesian_motion_generator_joint_velocity_discontinuity);
    robot_state_msg.set_current_errors_cartesian_motion_generator_joint_acceleration_discontinuity(robot_state_data->current_robot_state_.current_errors.cartesian_motion_generator_joint_acceleration_discontinuity);
    robot_state_msg.set_current_errors_cartesian_position_motion_generator_invalid_frame(robot_state_data->current_robot_state_.current_errors.cartesian_position_motion_generator_invalid_frame);
    robot_state_msg.set_current_errors_force_controller_desired_force_tolerance_violation(robot_state_data->current_robot_state_.current_errors.force_controller_desired_force_tolerance_violation);
    robot_state_msg.set_current_errors_controller_torque_discontinuity(robot_state_data->current_robot_state_.current_errors.controller_torque_discontinuity);
    robot_state_msg.set_current_errors_start_elbow_sign_inconsistent(robot_state_data->current_robot_state_.current_errors.start_elbow_sign_inconsistent);
    robot_state_msg.set_current_errors_communication_constraints_violation(robot_state_data->current_robot_state_.current_errors.communication_constraints_violation);
    robot_state_msg.set_current_errors_power_limit_violation(robot_state_data->current_robot_state_.current_errors.power_limit_violation);
    robot_state_msg.set_current_errors_joint_p2p_insufficient_torque_for_planning(robot_state_data->current_robot_state_.current_errors.joint_p2p_insufficient_torque_for_planning);
    robot_state_msg.set_current_errors_tau_j_range_violation(robot_state_data->current_robot_state_.current_errors.tau_j_range_violation);
    robot_state_msg.set_current_errors_instability_detected(robot_state_data->current_robot_state_.current_errors.instability_detected);
    robot_state_msg.set_current_errors_joint_move_in_wrong_direction(robot_state_data->current_robot_state_.current_errors.joint_move_in_wrong_direction);

    robot_state_msg.set_last_motion_errors_joint_position_limits_violation(robot_state_data->current_robot_state_.last_motion_errors.joint_position_limits_violation);
    robot_state_msg.set_last_motion_errors_cartesian_position_limits_violation(robot_state_data->current_robot_state_.last_motion_errors.cartesian_position_limits_violation);
    robot_state_msg.set_last_motion_errors_self_collision_avoidance_violation(robot_state_data->current_robot_state_.last_motion_errors.self_collision_avoidance_violation);
    robot_state_msg.set_last_motion_errors_joint_velocity_violation(robot_state_data->current_robot_state_.last_motion_errors.joint_velocity_violation);
    robot_state_msg.set_last_motion_errors_cartesian_velocity_violation(robot_state_data->current_robot_state_.last_motion_errors.cartesian_velocity_violation);
    robot_state_msg.set_last_motion_errors_force_control_safety_violation(robot_state_data->current_robot_state_.last_motion_errors.force_control_safety_violation);
    robot_state_msg.set_last_motion_errors_joint_reflex(robot_state_data->current_robot_state_.last_motion_errors.joint_reflex);
    robot_state_msg.set_last_motion_errors_cartesian_reflex(robot_state_data->current_robot_state_.last_motion_errors.cartesian_reflex);
    robot_state_msg.set_last_motion_errors_max_goal_pose_deviation_violation(robot_state_data->current_robot_state_.last_motion_errors.max_goal_pose_deviation_violation);
    robot_state_msg.set_last_motion_errors_max_path_pose_deviation_violation(robot_state_data->current_robot_state_.last_motion_errors.max_path_pose_deviation_violation);
    robot_state_msg.set_last_motion_errors_cartesian_velocity_profile_safety_violation(robot_state_data->current_robot_state_.last_motion_errors.cartesian_velocity_profile_safety_violation);
    robot_state_msg.set_last_motion_errors_joint_position_motion_generator_start_pose_invalid(robot_state_data->current_robot_state_.last_motion_errors.joint_position_motion_generator_start_pose_invalid);
    robot_state_msg.set_last_motion_errors_joint_motion_generator_position_limits_violation(robot_state_data->current_robot_state_.last_motion_errors.joint_motion_generator_position_limits_violation);
    robot_state_msg.set_last_motion_errors_joint_motion_generator_velocity_limits_violation(robot_state_data->current_robot_state_.last_motion_errors.joint_motion_generator_velocity_limits_violation);
    robot_state_msg.set_last_motion_errors_joint_motion_generator_velocity_discontinuity(robot_state_data->current_robot_state_.last_motion_errors.joint_motion_generator_velocity_discontinuity);
    robot_state_msg.set_last_motion_errors_joint_motion_generator_acceleration_discontinuity(robot_state_data->current_robot_state_.last_motion_errors.joint_motion_generator_acceleration_discontinuity);
    robot_state_msg.set_last_motion_errors_cartesian_position_motion_generator_start_pose_invalid(robot_state_data->current_robot_state_.last_motion_errors.cartesian_position_motion_generator_start_pose_invalid);
    robot_state_msg.set_last_motion_errors_cartesian_motion_generator_elbow_limit_violation(robot_state_data->current_robot_state_.last_motion_errors.cartesian_motion_generator_elbow_limit_violation);
    robot_state_msg.set_last_motion_errors_cartesian_motion_generator_velocity_limits_violation(robot_state_data->current_robot_state_.last_motion_errors.cartesian_motion_generator_velocity_limits_violation);
    robot_state_msg.set_last_motion_errors_cartesian_motion_generator_velocity_discontinuity(robot_state_data->current_robot_state_.last_motion_errors.cartesian_motion_generator_velocity_discontinuity);
    robot_state_msg.set_last_motion_errors_cartesian_motion_generator_acceleration_discontinuity(robot_state_data->current_robot_state_.last_motion_errors.cartesian_motion_generator_acceleration_discontinuity);
    robot_state_msg.set_last_motion_errors_cartesian_motion_generator_elbow_sign_inconsistent(robot_state_data->current_robot_state_.last_motion_errors.cartesian_motion_generator_elbow_sign_inconsistent);
    robot_state_msg.set_last_motion_errors_cartesian_motion_generator_start_elbow_invalid(robot_state_data->current_robot_state_.last_motion_errors.cartesian_motion_generator_start_elbow_invalid);
    robot_state_msg.set_last_motion_errors_cartesian_motion_generator_joint_position_limits_violation(robot_state_data->current_robot_state_.last_motion_errors.cartesian_motion_generator_joint_position_limits_violation);
    robot_state_msg.set_last_motion_errors_cartesian_motion_generator_joint_velocity_limits_violation(robot_state_data->current_robot_state_.last_motion_errors.cartesian_motion_generator_joint_velocity_limits_violation);
    robot_state_msg.set_last_motion_errors_cartesian_motion_generator_joint_velocity_discontinuity(robot_state_data->current_robot_state_.last_motion_errors.cartesian_motion_generator_joint_velocity_discontinuity);
    robot_state_msg.set_last_motion_errors_cartesian_motion_generator_joint_acceleration_discontinuity(robot_state_data->current_robot_state_.last_motion_errors.cartesian_motion_generator_joint_acceleration_discontinuity);
    robot_state_msg.set_last_motion_errors_cartesian_position_motion_generator_invalid_frame(robot_state_data->current_robot_state_.last_motion_errors.cartesian_position_motion_generator_invalid_frame);
    robot_state_msg.set_last_motion_errors_force_controller_desired_force_tolerance_violation(robot_state_data->current_robot_state_.last_motion_errors.force_controller_desired_force_tolerance_violation);
    robot_state_msg.set_last_motion_errors_controller_torque_discontinuity(robot_state_data->current_robot_state_.last_motion_errors.controller_torque_discontinuity);
    robot_state_msg.set_last_motion_errors_start_elbow_sign_inconsistent(robot_state_data->current_robot_state_.last_motion_errors.start_elbow_sign_inconsistent);
    robot_state_msg.set_last_motion_errors_communication_constraints_violation(robot_state_data->current_robot_state_.last_motion_errors.communication_constraints_violation);
    robot_state_msg.set_last_motion_errors_power_limit_violation(robot_state_data->current_robot_state_.last_motion_errors.power_limit_violation);
    robot_state_msg.set_last_motion_errors_joint_p2p_insufficient_torque_for_planning(robot_state_data->current_robot_state_.last_motion_errors.joint_p2p_insufficient_torque_for_planning);
    robot_state_msg.set_last_motion_errors_tau_j_range_violation(robot_state_data->current_robot_state_.last_motion_errors.tau_j_range_violation);
    robot_state_msg.set_last_motion_errors_instability_detected(robot_state_data->current_robot_state_.last_motion_errors.instability_detected);
    robot_state_msg.set_last_motion_errors_joint_move_in_wrong_direction(robot_state_data->current_robot_state_.last_motion_errors.joint_move_in_wrong_direction);

    robot_state_msg.set_control_command_success_rate(robot_state_data->current_robot_state_.control_command_success_rate);
    robot_state_msg.set_robot_mode(static_cast<RobotStateMessage::RobotMode>(robot_state_data->current_robot_state_.robot_mode));
    robot_state_msg.set_robot_time(robot_state_data->current_robot_state_.time.toSec());

    robot_state_msg.set_gripper_width(robot_state_data->current_gripper_state_.width);
    robot_state_msg.set_gripper_max_width(robot_state_data->current_gripper_state_.max_width);
    robot_state_msg.set_gripper_is_grasped(robot_state_data->current_gripper_state_.is_grasped);
    robot_state_msg.set_gripper_temperature(static_cast<uint32_t>(robot_state_data->current_gripper_state_.temperature));
    robot_state_msg.set_gripper_time(robot_state_data->current_gripper_state_.time.toSec());

    std::string robot_state_msg_string;

    robot_state_msg.SerializeToString(&robot_state_msg_string);

    int num_bytes = robot_state_msg_string.length();

    current_robot_state_buffer[0] = (num_bytes & 0xFF);
    current_robot_state_buffer[1] = ((num_bytes >> 8) & 0xFF);
    current_robot_state_buffer[2] = ((num_bytes >> 16) & 0xFF);
    current_robot_state_buffer[3] = ((num_bytes >> 24) & 0xFF);

    // Now mem copy all of the data in the form of bytes
    memcpy(current_robot_state_buffer + 4, &robot_state_msg_string[0], num_bytes * sizeof(uint8_t));

    shared_memory_handler->getCurrentRobotStateBufferMutex()->unlock();
  }
}