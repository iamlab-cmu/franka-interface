#ifndef FRANKA_INTERFACE_FEEDBACK_CONTROLLER_SET_INTERNAL_IMPEDANCE_FEEDBACK_CONTROLLER_H_
#define FRANKA_INTERFACE_FEEDBACK_CONTROLLER_SET_INTERNAL_IMPEDANCE_FEEDBACK_CONTROLLER_H_

#include "franka-interface/feedback_controller/feedback_controller.h"

class SetInternalImpedanceFeedbackController : public FeedbackController {
 public:
  using FeedbackController::FeedbackController;

  void parse_parameters() override;

  void initialize_controller(FrankaRobot *robot) override;

  void get_next_step(const franka::RobotState &robot_state, 
                     TrajectoryGenerator *traj_generator) override;

  bool set_cartesian_impedance_ = false;
  bool set_joint_impedance_ = false;

 private:
  InternalImpedanceFeedbackControllerMessage internal_impedance_feedback_params_;

  // Max Joint and Cartesian Impedance Values. 
  // TODO: Check to see what they actually are.
  // Kevin simply guessed 10000.0
  double max_cartesian_impedance_ = 10000.0;
  double max_joint_impedance_ = 10000.0;
  

  std::array<double, 6> cartesian_impedances_ = {{3000, 3000, 3000, 300, 300, 300}};
  std::array<double, 7> joint_impedances_ = {{3000, 3000, 3000, 2500, 2500, 2000, 2000}};
};

#endif  // FRANKA_INTERFACE_FEEDBACK_CONTROLLER_SET_INTERNAL_IMPEDANCE_FEEDBACK_CONTROLLER_H_