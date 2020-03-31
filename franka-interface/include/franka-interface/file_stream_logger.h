#ifndef FRANKA_INTERFACE_FILE_STREAM_LOGGER_H_
#define FRANKA_INTERFACE_FILE_STREAM_LOGGER_H_

#include <array>
#include <fstream>
#include <vector>

class FileStreamLogger {
 public:
  FileStreamLogger(const std::string& filename) : 
                   filename_(filename),
                   open_file_stream_(filename, std::ofstream::out | std::ofstream::app) 
  {};

  bool write_pose_desired_ = true;
  bool write_F_T_EE_ = true;
  bool write_m_ee_ = true;
  bool write_I_ee = true;
  bool write_F_x_Cee = true;
  bool write_m_load_ = true;
  bool write_I_load = true;
  bool write_F_x_Cload = true;
  bool write_m_total_ = true;
  bool write_I_total = true;
  bool write_F_x_Ctotal = true;
  bool write_joint_collision_=true;
  bool write_cartesian_collision_=true;
  bool write_current_errors_=true;
  bool write_last_motion_errors_=true;
  bool write_gripper_max_width_=true;

  bool writeData(std::vector<double>& time_since_skill_started_vector,
                 std::vector<std::array<double, 16>>& pose_desired_vector,
                 std::vector<std::array<double, 16>>& O_T_EE_vector,
                 std::vector<std::array<double, 16>>& O_T_EE_d_vector,
                 std::vector<std::array<double, 16>>& F_T_EE_vector,
                 std::vector<std::array<double, 16>>& EE_T_K_vector,
                 std::vector<double>& m_ee_vector,
                 std::vector<std::array<double, 9>>& I_ee_vector,
                 std::vector<std::array<double, 3>>& F_x_Cee_vector,
                 std::vector<double>& m_load_vector,
                 std::vector<std::array<double, 9>>& I_load_vector,
                 std::vector<std::array<double, 3>>& F_x_Cload_vector,
                 std::vector<double>& m_total_vector,
                 std::vector<std::array<double, 9>>& I_total_vector,
                 std::vector<std::array<double, 3>>& F_x_Ctotal_vector,
                 std::vector<std::array<double, 2>>& elbow_vector,
                 std::vector<std::array<double, 2>>& elbow_d_vector,
                 std::vector<std::array<double, 2>>& elbow_c_vector,
                 std::vector<std::array<double, 2>>& delbow_c_vector,
                 std::vector<std::array<double, 2>>& ddelbow_c_vector,
                 std::vector<std::array<double, 7>>& tau_J_vector,
                 std::vector<std::array<double, 7>>& tau_J_d_vector,
                 std::vector<std::array<double, 7>>& dtau_J_vector,
                 std::vector<std::array<double, 7>>& q_vector,
                 std::vector<std::array<double, 7>>& q_d_vector,
                 std::vector<std::array<double, 7>>& dq_vector,
                 std::vector<std::array<double, 7>>& dq_d_vector,
                 std::vector<std::array<double, 7>>& ddq_d_vector,
                 std::vector<std::array<double, 7>>& joint_contact_vector,
                 std::vector<std::array<double, 6>>& cartesian_contact_vector,
                 std::vector<std::array<double, 7>>& joint_collision_vector,
                 std::vector<std::array<double, 6>>& cartesian_collision_vector,
                 std::vector<std::array<double, 7>>& tau_ext_hat_filtered_vector,
                 std::vector<std::array<double, 6>>& O_F_ext_hat_K_vector,
                 std::vector<std::array<double, 6>>& K_F_ext_hat_K_vector,
                 std::vector<std::array<double, 6>>& O_dP_EE_d_vector,
                 std::vector<std::array<double, 16>>& O_T_EE_c_vector,
                 std::vector<std::array<double, 6>>& O_dP_EE_c_vector,
                 std::vector<std::array<double, 6>>& O_ddP_EE_c_vector,
                 std::vector<std::array<double, 7>>& theta_vector,
                 std::vector<std::array<double, 7>>& dtheta_vector,
                 std::vector<std::array<double, 144>>& frames_vector,
                 std::vector<std::array<bool, 37>>& current_errors_vector,
                 std::vector<std::array<bool, 37>>& last_motion_errors_vector,
                 std::vector<double>& control_command_success_rate_vector,
                 std::vector<uint8_t>& robot_mode_vector,
                 std::vector<double>& robot_time_vector,
                 std::vector<double>& gripper_width_vector,
                 std::vector<double>& gripper_max_width_vector,
                 std::vector<bool>& gripper_is_grasped_vector,
                 std::vector<uint16_t>& gripper_temperature_vector,
                 std::vector<double>& gripper_time_vector);

  /**
   * Write string data to logger. String data is prefixed by "info:" to allow us 
   * to easily find it in the logs.
   * @param data String data to log does not require a "\n" in the end.
   * @return True if we did write the data successfully else false.
   */
  bool writeStringData(std::vector<std::string> data);

  /**
   * Update logger filename.
   *
   * @param new_filename
   * @return
   */
  void updateFileName(std::string new_filename);

  /**
   * Initialize the first line of the file with the parameters that are being saved. 
   */
  void initializeFile();

 private:
  std::string filename_;
  std::ofstream open_file_stream_;
};

#endif  // FRANKA_INTERFACE_FILE_STREAM_LOGGER_H_