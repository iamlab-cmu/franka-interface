#ifndef FRANKA_INTERFACE_SKILLS_CARTESIAN_POSE_SKILL_H_
#define FRANKA_INTERFACE_SKILLS_CARTESIAN_POSE_SKILL_H_

#include "franka-interface/skills/base_skill.h"

class CartesianPoseSkill : public BaseSkill {
 public:
  CartesianPoseSkill(int skill_idx, int meta_skill_idx, std::string description) : 
                              BaseSkill(skill_idx, meta_skill_idx, description)
  {
    for(int i = 0; i < 3; i++) {
      previous_velocity_[i] = 0.0;
      previous_acceleration_[i] = 0.0;
      current_velocity_[i] = 0.0;
      current_acceleration_[i] = 0.0;
      current_jerk_[i] = 0.0;
      previous_error_[i] = 0.0;
    }
    
  };

  std::array<double, 16> limit_position(std::array<double, 16> &desired_pose, double period);

  std::array<double, 16> limit_position_to_stop(std::array<double, 16> &current_pose, double period);

  void execute_skill_on_franka(run_loop* run_loop,
                               FrankaRobot* robot,
                               FrankaGripper* gripper,
                               RobotStateData* robot_state_data) override;

 private:
  bool return_status_{false};

  std::array<double, 16> previous_desired_pose_;

  std::array<double, 3> current_error_;
  std::array<double, 3> previous_error_;
  std::array<double, 3> integral_;
  std::array<double, 3> derivative_;

  std::array<double, 3> previous_position_;
  std::array<double, 3> previous_velocity_;
  std::array<double, 3> previous_acceleration_;

  std::array<double, 3> current_position_;
  std::array<double, 3> current_velocity_;
  std::array<double, 3> current_acceleration_;
  std::array<double, 3> current_jerk_;

  std::array<double, 3> next_position_;
  std::array<double, 3> next_velocity_;
  std::array<double, 3> next_acceleration_;
  std::array<double, 3> next_jerk_;

  double safety_factor = 0.1;
  double Kp_ = 30.0;
  double Ki_ = 0.0; // 0.1;
  double Kd_ = 0.01;
  double eps_ = 0.0001;

  // Franka Parameters from https://frankaemika.github.io/docs/control_parameters.html
  const double max_cartesian_translation_velocity_ = 1.7; // m / s
  const double max_cartesian_rotation_velocity_ = 2.5; // rad / s
  const double max_cartesian_elbow_velocity_ = 2.175; // rad / s
  const double max_cartesian_translation_acceleration_ = 13.0; // m / s^2
  const double max_cartesian_rotation_acceleration_ = 25.0; // rad / s^2
  const double max_cartesian_elbow_acceleration_ = 10.0; // rad / s^2
  const double max_cartesian_translation_jerk_ = 6500.0; // m / s^3
  const double max_cartesian_rotation_jerk_ = 12500.0; // rad / s^3
  const double max_cartesian_elbow_jerk_ = 5000.0; // rad / s^3
};

#endif  // FRANKA_INTERFACE_SKILLS_CARTESIAN_POSE_SKILL_H_