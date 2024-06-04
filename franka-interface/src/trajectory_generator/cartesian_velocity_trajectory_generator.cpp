#include "franka-interface/trajectory_generator/cartesian_velocity_trajectory_generator.h"

void CartesianVelocityTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));
  bool parsed_params = cartesian_velocity_trajectory_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    run_time_ = cartesian_velocity_trajectory_params_.run_time();
    for(int i = 0; i < 6; i++) {
      goal_cartesian_velocities_[i] = cartesian_velocity_trajectory_params_.cartesian_velocities(i);
      cartesian_accelerations_[i] = cartesian_velocity_trajectory_params_.cartesian_accelerations(i);
    }
  } else {
    std::cout << "Parsing CartesianVelocityTrajectoryGenerator params failed. Data size = " << data_size << std::endl;
  }
}

void CartesianVelocityTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state,
                                                             SkillType skill_type) {
  initialize_initial_cartesian_velocities(robot_state, skill_type);
}

void CartesianVelocityTrajectoryGenerator::initialize_initial_cartesian_velocities(const franka::RobotState &robot_state,
                                                                           SkillType skill_type) {
  switch(skill_type) {
    case SkillType::CartesianVelocitySkill:
      initial_cartesian_velocities_ = robot_state.O_dP_EE_d;
      desired_cartesian_velocities_ = robot_state.O_dP_EE_d;
      break;
    default:
      initial_cartesian_velocities_ = robot_state.O_dP_EE_d;
      desired_cartesian_velocities_ = robot_state.O_dP_EE_d;
  }
}

const std::array<double, 6>& CartesianVelocityTrajectoryGenerator::get_desired_cartesian_velocities() const {
  return desired_cartesian_velocities_;
}
