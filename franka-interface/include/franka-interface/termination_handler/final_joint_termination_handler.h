#ifndef FRANKA_INTERFACE_TERMINATION_HANDLER_FINAL_JOINT_TERMINATION_HANDLER_H_
#define FRANKA_INTERFACE_TERMINATION_HANDLER_FINAL_JOINT_TERMINATION_HANDLER_H_

#include "franka-interface/termination_handler/termination_handler.h"

class FinalJointTerminationHandler :public TerminationHandler{
 public:
  using TerminationHandler::TerminationHandler;

  /**
   * Parse parameters from memory.
   */
  void parse_parameters() override;

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

 protected:
  JointThresholdMessage joint_termination_params_;

  double default_joint_threshold_ = 0.0001;
  std::array<double, 7> joint_thresholds_;
};

#endif  // FRANKA_INTERFACE_TERMINATION_HANDLER_FINAL_JOINT_TERMINATION_HANDLER_H_