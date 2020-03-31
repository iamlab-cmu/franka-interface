//
// Created by mohit on 11/25/18.
//

#include "franka-interface/feedback_controller/noop_feedback_controller.h"

void NoopFeedbackController::parse_parameters() {
  // pass
}

void NoopFeedbackController::initialize_controller(FrankaRobot *robot) {
  // pass
}

void NoopFeedbackController::get_next_step(const franka::RobotState &robot_state, 
                                           TrajectoryGenerator *traj_generator) {
  // pass
}