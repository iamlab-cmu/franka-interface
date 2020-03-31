#ifndef FRANKA_INTERFACE_TERMINATION_HANDLER_FINAL_POSE_TERMINATION_HANDLER_H_
#define FRANKA_INTERFACE_TERMINATION_HANDLER_FINAL_POSE_TERMINATION_HANDLER_H_

#include <Eigen/Dense>

#include "franka-interface/termination_handler/termination_handler.h"

class FinalPoseTerminationHandler : public TerminationHandler {
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
  virtual bool should_terminate(const franka::RobotState &robot_state, 
                                          franka::Model *robot_model,
                                          TrajectoryGenerator *traj_generator) override;

 private:
  PoseThresholdMessage pose_termination_params_;
  double default_position_threshold_ = 0.0001;
  double default_orientation_threshold_ = 0.0001;
  Eigen::Vector3d position_thresholds_;
  Eigen::Vector3d orientation_thresholds_;
};

#endif  // FRANKA_INTERFACE_TERMINATION_HANDLER_FINAL_POSE_TERMINATION_HANDLER_H_