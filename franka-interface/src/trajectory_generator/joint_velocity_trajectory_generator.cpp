#include "franka-interface/trajectory_generator/joint_velocity_trajectory_generator.h"

void JointVelocityTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));
  bool parsed_params = joint_velocity_trajectory_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    run_time_ = joint_velocity_trajectory_params_.run_time();
    for(int i = 0; i < 7; i++) {
      goal_joint_velocities_[i] = joint_velocity_trajectory_params_.joint_velocities(i);
      joint_accelerations_[i] = joint_velocity_trajectory_params_.joint_accelerations(i);
    }
  } else {
    std::cout << "Parsing JointVelocityTrajectoryGenerator params failed. Data size = " << data_size << std::endl;
  }
}

void JointVelocityTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state,
                                                             SkillType skill_type) {
  initialize_initial_joint_velocities(robot_state, skill_type);
}

void JointVelocityTrajectoryGenerator::initialize_initial_joint_velocities(const franka::RobotState &robot_state,
                                                                           SkillType skill_type) {
  switch(skill_type) {
    case SkillType::JointPositionSkill:
      initial_joint_velocities_ = robot_state.dq_d;
      desired_joint_velocities_ = robot_state.dq_d;
      break;
    case SkillType::ImpedanceControlSkill:
      initial_joint_velocities_ = robot_state.dq;
      desired_joint_velocities_ = robot_state.dq;
      break;
    default:
      initial_joint_velocities_ = robot_state.dq_d;
      desired_joint_velocities_ = robot_state.dq_d;
  }
}

const std::array<double, 7>& JointVelocityTrajectoryGenerator::get_desired_joint_velocities() const {
  return desired_joint_velocities_;
}
