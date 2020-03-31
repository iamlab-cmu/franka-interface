#ifndef FRANKA_INTERFACE_FEEDBACK_CONTROLLER_NOOP_FEEDBACK_CONTROLLER_H_
#define FRANKA_INTERFACE_FEEDBACK_CONTROLLER_NOOP_FEEDBACK_CONTROLLER_H_

#include "franka-interface/feedback_controller/feedback_controller.h"

class NoopFeedbackController : public FeedbackController {
 public:
  using FeedbackController::FeedbackController;

  void parse_parameters() override;

  void initialize_controller(FrankaRobot *robot) override;

  void get_next_step(const franka::RobotState &robot_state, 
                     TrajectoryGenerator *traj_generator) override;
};

#endif  // FRANKA_INTERFACE_FEEDBACK_CONTROLLER_NOOP_FEEDBACK_CONTROLLER_H_