//
// Created by mohit on 11/26/18.
//

#include "franka-interface/termination_handler/time_termination_handler.h"

void TimeTerminationHandler::parse_parameters() {
  
  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = time_termination_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    buffer_time_ = time_termination_params_.buffer_time();

  } else {
    std::cout << "Parsing TimeTerminationHandler params failed. Data size = " << data_size << std::endl;
  }
}

bool TimeTerminationHandler::should_terminate(const franka::RobotState &robot_state, 
                                                        franka::Model *model,
                                                        TrajectoryGenerator *trajectory_generator) {
  check_terminate_virtual_wall_collisions(robot_state, model);
  check_terminate_preempt();
  check_terminate_time(trajectory_generator);
  
  return done_;
}
