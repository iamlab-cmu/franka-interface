//
// Created by kevin on 3/27/19.
//

#include "franka-interface/feedback_controller/set_internal_impedance_feedback_controller.h"

void SetInternalImpedanceFeedbackController::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = internal_impedance_feedback_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){

    if(internal_impedance_feedback_params_.cartesian_impedances_size() == 6){
      for(size_t i = 0; i < 7; i++) {
        cartesian_impedances_[i] = internal_impedance_feedback_params_.cartesian_impedances(i);
      }
      set_cartesian_impedance_ = true;
    }

    if(internal_impedance_feedback_params_.joint_impedances_size() == 7){
      for(size_t i = 0; i < 7; i++) {
        joint_impedances_[i] = internal_impedance_feedback_params_.joint_impedances(i);
      }
      set_joint_impedance_ = true;
    }
  } else {
    std::cout << "Parsing SetInternalImpedanceFeedbackController params failed. Data size = " << data_size << std::endl;
  }
}

void SetInternalImpedanceFeedbackController::initialize_controller(FrankaRobot *robot) {
  
  if(set_cartesian_impedance_) {
    bool cartesian_impedance_values_valid = true;
    for(size_t i = 0; i < cartesian_impedances_.size(); i++) {
      if(cartesian_impedances_[i] < 0.0 or cartesian_impedances_[i] > max_cartesian_impedance_) {
        cartesian_impedance_values_valid = false;
      }
    }
    if(cartesian_impedance_values_valid) {
      robot->setCartesianImpedance(cartesian_impedances_);
    }
  }

  if(set_joint_impedance_) {
    bool joint_impedance_values_valid = true;
    for(size_t i = 0; i < joint_impedances_.size(); i++) {
      if(joint_impedances_[i] < 0.0 or joint_impedances_[i] > max_joint_impedance_) {
        joint_impedance_values_valid = false;
      }
    }
    if(joint_impedance_values_valid) {
      robot->setJointImpedance(joint_impedances_);
    }
  }
}

void SetInternalImpedanceFeedbackController::get_next_step(const franka::RobotState &robot_state, 
                                                           TrajectoryGenerator *traj_generator) {
  // pass
}