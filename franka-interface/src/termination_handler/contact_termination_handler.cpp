//
// Created by mohit on 11/26/18.
//

#include "franka-interface/termination_handler/contact_termination_handler.h"

#include <cmath>

#include "franka-interface/trajectory_generator/trajectory_generator.h"

void ContactTerminationHandler::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = contact_termination_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    buffer_time_ = contact_termination_params_.buffer_time();

    if (contact_termination_params_.force_thresholds_size() == 6) {
      for (int i = 0; i < 6; i++) {
        lower_force_thresholds_acceleration_[i] = contact_termination_params_.force_thresholds(i);
        lower_force_thresholds_nominal_[i] = contact_termination_params_.force_thresholds(i);
      }
    } else {
      lower_force_thresholds_acceleration_ = default_lower_force_thresholds_acceleration_;
      lower_force_thresholds_nominal_ = default_lower_force_thresholds_nominal_;
    }

    if (contact_termination_params_.torque_thresholds_size() == 7) {
      for (int i = 0; i < 7; i++) {
        lower_torque_thresholds_acceleration_[i] = contact_termination_params_.torque_thresholds(i);
        lower_torque_thresholds_nominal_[i] = contact_termination_params_.torque_thresholds(i);
      }
    } else {
      lower_torque_thresholds_acceleration_ = default_lower_torque_thresholds_acceleration_;
      lower_torque_thresholds_nominal_ = default_lower_torque_thresholds_nominal_;
    }

    upper_torque_thresholds_acceleration_ = default_upper_torque_thresholds_acceleration_;
    upper_torque_thresholds_nominal_ = default_upper_torque_thresholds_nominal_;
    upper_force_thresholds_acceleration_ = default_upper_force_thresholds_acceleration_;
    upper_force_thresholds_nominal_ = default_upper_force_thresholds_nominal_;
  } else {
    std::cout << "Parsing ContactTerminationHandler params failed. Data size = " << data_size << std::endl;
  }
}

void ContactTerminationHandler::initialize_handler(FrankaRobot *robot) {
  robot->robot_.setCollisionBehavior(lower_torque_thresholds_acceleration_, 
                              upper_torque_thresholds_acceleration_,
                              lower_torque_thresholds_nominal_,
                              upper_torque_thresholds_nominal_,
                              lower_force_thresholds_acceleration_,
                              upper_force_thresholds_acceleration_,
                              lower_force_thresholds_nominal_,
                              upper_force_thresholds_nominal_);
}

bool ContactTerminationHandler::should_terminate(const franka::RobotState &robot_state,
                                                            franka::Model *model,
                                                           TrajectoryGenerator *trajectory_generator) {
  check_terminate_preempt();
  check_terminate_virtual_wall_collisions(robot_state, model);
  check_terminate_time(trajectory_generator);

  if(!done_) {

    std::array<double, 6> cartesian_contact = robot_state.cartesian_contact;

    for(int i = 0; i < 6; i++) {
      if(cartesian_contacts_to_use_[i] != 0 && cartesian_contact[i] != 0) {
        done_ = true;
        return done_;
      }
    }
  }
  
  return done_;
}
