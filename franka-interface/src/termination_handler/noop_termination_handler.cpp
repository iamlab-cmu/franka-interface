//
// Created by mohit on 11/26/18.
//

#include "franka-interface/termination_handler/noop_termination_handler.h"

bool NoopTerminationHandler::should_terminate(const franka::RobotState &robot_state, 
                                                        franka::Model *model,
                                                        TrajectoryGenerator *trajectory_generator) {
  check_terminate_preempt();
  check_terminate_virtual_wall_collisions(robot_state, model);

  return false;
}
