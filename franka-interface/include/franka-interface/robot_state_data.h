#ifndef FRANKA_INTERFACE_ROBOT_STATE_DATA_H_
#define FRANKA_INTERFACE_ROBOT_STATE_DATA_H_

#include <array>
#include <atomic>
#include <mutex>
#include <vector>
#include <thread>

#include <franka/robot.h>
#include <franka/gripper.h>

class FileStreamLogger;

class RobotStateData {
 public:
  std::atomic<bool> use_buffer_0;
  std::mutex buffer_0_mutex_;
  std::mutex buffer_1_mutex_;

  RobotStateData(std::mutex &m): mutex_(m) {};

  std::mutex& mutex_;
  bool has_data_=false;

  double time_=0;
  int counter_=0;

  // Log skill info e.g. name, id, type of skill before it begins.
  std::vector<std::string> log_skill_info_0_{};
  std::vector<std::string> log_skill_info_1_{};

  std::vector<double> log_time_since_skill_started_0_{};

  
  std::vector<std::array<double, 16>> log_pose_desired_0_{};
  std::vector<std::array<double, 16>> log_O_T_EE_0_{};
  std::vector<std::array<double, 16>> log_O_T_EE_d_0_{};
  std::vector<std::array<double, 16>> log_F_T_EE_0_{};
  std::vector<std::array<double, 16>> log_EE_T_K_0_{};
  std::vector<double> log_m_ee_0_{};
  std::vector<std::array<double, 9>> log_I_ee_0_{};
  std::vector<std::array<double, 3>> log_F_x_Cee_0_{};
  std::vector<double> log_m_load_0_{};
  std::vector<std::array<double, 9>> log_I_load_0_{};
  std::vector<std::array<double, 3>> log_F_x_Cload_0_{};
  std::vector<double> log_m_total_0_{};
  std::vector<std::array<double, 9>> log_I_total_0_{};
  std::vector<std::array<double, 3>> log_F_x_Ctotal_0_{};
  std::vector<std::array<double, 2>> log_elbow_0_{};
  std::vector<std::array<double, 2>> log_elbow_d_0_{};
  std::vector<std::array<double, 2>> log_elbow_c_0_{};
  std::vector<std::array<double, 2>> log_delbow_c_0_{};
  std::vector<std::array<double, 2>> log_ddelbow_c_0_{};
  std::vector<std::array<double, 7>> log_tau_J_0_{};
  std::vector<std::array<double, 7>> log_tau_J_d_0_{};
  std::vector<std::array<double, 7>> log_dtau_J_0_{};
  std::vector<std::array<double, 7>> log_q_0_{};
  std::vector<std::array<double, 7>> log_q_d_0_{};
  std::vector<std::array<double, 7>> log_dq_0_{};
  std::vector<std::array<double, 7>> log_dq_d_0_{};
  std::vector<std::array<double, 7>> log_ddq_d_0_{};
  std::vector<std::array<double, 7>> log_joint_contact_0_{};
  std::vector<std::array<double, 6>> log_cartesian_contact_0_{};
  std::vector<std::array<double, 7>> log_joint_collision_0_{};
  std::vector<std::array<double, 6>> log_cartesian_collision_0_{};
  std::vector<std::array<double, 7>> log_tau_ext_hat_filtered_0_{};
  std::vector<std::array<double, 6>> log_O_F_ext_hat_K_0_{};
  std::vector<std::array<double, 6>> log_K_F_ext_hat_K_0_{};
  std::vector<std::array<double, 6>> log_O_dP_EE_d_0_{};
  std::vector<std::array<double, 16>> log_O_T_EE_c_0_{};
  std::vector<std::array<double, 6>> log_O_dP_EE_c_0_{};
  std::vector<std::array<double, 6>> log_O_ddP_EE_c_0_{};
  std::vector<std::array<double, 7>> log_theta_0_{};
  std::vector<std::array<double, 7>> log_dtheta_0_{};
  std::vector<std::array<double, 144>> log_frames_0_{};
  std::vector<std::array<bool, 37>> log_current_errors_0_{};
  std::vector<std::array<bool, 37>> log_last_motion_errors_0_{};
  std::vector<double> log_control_command_success_rate_0_{};
  std::vector<uint8_t> log_robot_mode_0_{};
  std::vector<double> log_robot_time_0_{};

  std::vector<double> log_gripper_width_0_{};
  std::vector<double> log_gripper_max_width_0_{};
  std::vector<bool> log_gripper_is_grasped_0_{};
  std::vector<uint16_t> log_gripper_temperature_0_{};
  std::vector<double> log_gripper_time_0_{};

  std::vector<double> log_time_since_skill_started_1_{};

  std::vector<std::array<double, 16>> log_pose_desired_1_{};
  std::vector<std::array<double, 16>> log_O_T_EE_1_{};
  std::vector<std::array<double, 16>> log_O_T_EE_d_1_{};
  std::vector<std::array<double, 16>> log_F_T_EE_1_{};
  std::vector<std::array<double, 16>> log_EE_T_K_1_{};
  std::vector<double> log_m_ee_1_{};
  std::vector<std::array<double, 9>> log_I_ee_1_{};
  std::vector<std::array<double, 3>> log_F_x_Cee_1_{};
  std::vector<double> log_m_load_1_{};
  std::vector<std::array<double, 9>> log_I_load_1_{};
  std::vector<std::array<double, 3>> log_F_x_Cload_1_{};
  std::vector<double> log_m_total_1_{};
  std::vector<std::array<double, 9>> log_I_total_1_{};
  std::vector<std::array<double, 3>> log_F_x_Ctotal_1_{};
  std::vector<std::array<double, 2>> log_elbow_1_{};
  std::vector<std::array<double, 2>> log_elbow_d_1_{};
  std::vector<std::array<double, 2>> log_elbow_c_1_{};
  std::vector<std::array<double, 2>> log_delbow_c_1_{};
  std::vector<std::array<double, 2>> log_ddelbow_c_1_{};
  std::vector<std::array<double, 7>> log_tau_J_1_{};
  std::vector<std::array<double, 7>> log_tau_J_d_1_{};
  std::vector<std::array<double, 7>> log_dtau_J_1_{};
  std::vector<std::array<double, 7>> log_q_1_{};
  std::vector<std::array<double, 7>> log_q_d_1_{};
  std::vector<std::array<double, 7>> log_dq_1_{};
  std::vector<std::array<double, 7>> log_dq_d_1_{};
  std::vector<std::array<double, 7>> log_ddq_d_1_{};
  std::vector<std::array<double, 7>> log_joint_contact_1_{};
  std::vector<std::array<double, 6>> log_cartesian_contact_1_{};
  std::vector<std::array<double, 7>> log_joint_collision_1_{};
  std::vector<std::array<double, 6>> log_cartesian_collision_1_{};
  std::vector<std::array<double, 7>> log_tau_ext_hat_filtered_1_{};
  std::vector<std::array<double, 6>> log_O_F_ext_hat_K_1_{};
  std::vector<std::array<double, 6>> log_K_F_ext_hat_K_1_{};
  std::vector<std::array<double, 6>> log_O_dP_EE_d_1_{};
  std::vector<std::array<double, 16>> log_O_T_EE_c_1_{};
  std::vector<std::array<double, 6>> log_O_dP_EE_c_1_{};
  std::vector<std::array<double, 6>> log_O_ddP_EE_c_1_{};
  std::vector<std::array<double, 7>> log_theta_1_{};
  std::vector<std::array<double, 7>> log_dtheta_1_{};
  std::vector<std::array<double, 144>> log_frames_1_{};
  std::vector<std::array<bool, 37>> log_current_errors_1_{};
  std::vector<std::array<bool, 37>> log_last_motion_errors_1_{};
  std::vector<double> log_control_command_success_rate_1_{};
  std::vector<uint8_t> log_robot_mode_1_{};
  std::vector<double> log_robot_time_1_{};
  
  std::vector<double> log_gripper_width_1_{};
  std::vector<double> log_gripper_max_width_1_{};
  std::vector<bool> log_gripper_is_grasped_1_{};
  std::vector<uint16_t> log_gripper_temperature_1_{};
  std::vector<double> log_gripper_time_1_{};

  double current_gripper_width_{-1.0};
  double current_gripper_max_width_{-1.0};
  bool current_gripper_is_grasped_{false};
  uint16_t current_gripper_temperature_{0};
  double current_gripper_time_{-1.0};

  franka::RobotState current_robot_state_;
  franka::GripperState current_gripper_state_;
  std::array<double, 144> current_robot_frames_;

  std::array<double, 16> current_pose_desired_;

  // Utils for printing
  const int print_rate_=10;

  const int log_rate_=10;

  std::thread file_logger_thread_;

  /**
   * Set filestream logger to save data.
   * @param logger
   */
  void setFileStreamLogger(FileStreamLogger *logger);

  /**
   * Update filestream logger log file.  This method is called after every reset.
   *
   * @param new_filename New file to save logs in.
   */
  void updateFileStreamLogger(std::string new_filename);

  /**
   * Start logging to some global buffer or file.
   */
  void startFileLoggerThread();

  /**
   * Force write everything in current buffer to global buffer.
   * Use this to make sure we do not lose any data when we crash.
   */
  void writeCurrentBufferData();

  /**
   * Print Data beginning from end.
   * @param print_count
   */
  void printData(int print_count);

  /*
   * Helper methods for logging.
   */
  void log_robot_state(std::array<double, 16> &desired_pose, 
                       franka::RobotState robot_state,
                       franka::Model *robot_model, 
                       double time_since_skill_started);

  void update_current_gripper_state(franka::GripperState gripper_state);
  void log_skill_info(std::string skill_info);

  void clearAllBuffers();

 private:
  FileStreamLogger *file_logger_ = nullptr;

  void writeBufferData_0();

  void writeBufferData_1();
};

#endif  // FRANKA_INTERFACE_ROBOT_STATE_DATA_H_