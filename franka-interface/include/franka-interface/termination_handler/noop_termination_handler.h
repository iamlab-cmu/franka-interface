#ifndef FRANKA_INTERFACE_TERMINATION_HANDLER_NOOP_TERMINATION_HANDLER_H_
#define FRANKA_INTERFACE_TERMINATION_HANDLER_NOOP_TERMINATION_HANDLER_H_

#include "franka-interface/termination_handler/termination_handler.h"

class NoopTerminationHandler : public TerminationHandler {
 public:
  using TerminationHandler::TerminationHandler;

  /**
   * Parse parameters from memory.
   */
  void parse_parameters() {};

  /**
   * Initialize termination handler after parameter parsing.
   */
  void initialize_handler(FrankaRobot *robot) {};

  /**
   * Should we terminate the current skill.
   */
  bool should_terminate(const franka::RobotState &robot_state, 
                                  franka::Model *robot_model,
                                  TrajectoryGenerator *traj_generator) override;

};

#endif  // FRANKA_INTERFACE_TERMINATION_HANDLER_NOOP_TERMINATION_HANDLER_H_