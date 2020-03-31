#ifndef FRANKA_INTERFACE_FEEDBACK_CONTROLLER_JOINT_IMPEDANCE_FEEDBACK_CONTROLLER_H_
#define FRANKA_INTERFACE_FEEDBACK_CONTROLLER_JOINT_IMPEDANCE_FEEDBACK_CONTROLLER_H_

#include "franka-interface/feedback_controller/feedback_controller.h"

#include <array>

class JointImpedanceFeedbackController : public FeedbackController {
 public:
  using FeedbackController::FeedbackController;

  void parse_parameters() override;

  void initialize_controller(FrankaRobot *robot) override;

  void get_next_step(const franka::RobotState &robot_state, TrajectoryGenerator *traj_generator) override;

 private:
  JointImpedanceFeedbackControllerMessage joint_impedance_feedback_params_;

  const franka::Model *model_;
  // Stiffness
  std::array<double, 7> k_gains_ = {{600.0, 600.0, 600.0, 600.0, 
                                     250.0, 150.0, 50.0}};
  // Damping
  std::array<double, 7> d_gains_ = {{50.0, 50.0, 50.0, 50.0, 
                                     30.0, 25.0, 15.0}};
};

#endif  // FRANKA_INTERFACE_FEEDBACK_CONTROLLER_JOINT_IMPEDANCE_FEEDBACK_CONTROLLER_H_