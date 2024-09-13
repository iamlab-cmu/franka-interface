//
// Created by Kevin on 9/13/24.
//

#include "franka-interface/feedback_controller/pass_through_joint_torque_feedback_controller.h"

#include "franka-interface/trajectory_generator/impulse_trajectory_generator.h"

void PassThroughJointTorqueFeedbackController::parse_parameters() {
  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = joint_torque_feedback_params_.ParseFromArray(params_ + 5, data_size);

  if (parsed_params){
    for (int i = 0; i < 7; i++) {
        S_[i] = std::min(std::max(joint_torque_feedback_params_.selection(i), 0.), 1.);
        remove_gravity_[i] = joint_torque_feedback_params_.remove_gravity(i);
        desired_joint_torques_[i] = joint_torque_feedback_params_.joint_torques(i);
    }
  } else {
    std::cout << "Parsing JointTorqueFeedbackController params failed. Data size = " << data_size << std::endl;
  }
}

void PassThroughJointTorqueFeedbackController::initialize_controller(FrankaRobot *robot) {
  model_ = robot->getModel();
}

void PassThroughJointTorqueFeedbackController::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readFeedbackControllerSensorMessage(joint_torque_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
    for (int i = 0; i < 7; i++) {
        S_[i] = std::min(std::max(joint_torque_sensor_msg_.selection(i), 0.), 1.);
        remove_gravity_[i] = joint_torque_sensor_msg_.remove_gravity(i);
        desired_joint_torques_[i] = joint_torque_sensor_msg_.joint_torques(i);
    }
  }
}

void PassThroughJointTorqueFeedbackController::get_next_step(const franka::RobotState &robot_state,
                                                  TrajectoryGenerator *traj_generator) {
    
  std::array<double, 7> gravity = model_->gravity(robot_state);

  for (int i = 0; i < 7; i++) {
    if (S_[i] == 1){
        tau_d_array_[i] = desired_joint_torques_[i];
    }
    if (remove_gravity_[i] == 1){
        tau_d_array_[i] -= gravity[i];
    }
  }
}