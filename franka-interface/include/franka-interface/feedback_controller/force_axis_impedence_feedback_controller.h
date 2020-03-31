#ifndef FRANKA_INTERFACE_FEEDBACK_CONTROLLER_FORCE_AXIS_IMPEDENCE_FEEDBACK_CONTROLLER_H_
#define FRANKA_INTERFACE_FEEDBACK_CONTROLLER_FORCE_AXIS_IMPEDENCE_FEEDBACK_CONTROLLER_H_

#include <Eigen/Dense>

#include "franka-interface/feedback_controller/feedback_controller.h"

// An impedence controller that allows applying a force along a constrained 
// axis.
class ForceAxisImpedenceFeedbackController : public FeedbackController {
 public:
  using FeedbackController::FeedbackController;

  void parse_parameters() override;

  void initialize_controller(FrankaRobot *robot) override;

  void get_next_step(const franka::RobotState &robot_state, 
                     TrajectoryGenerator *traj_generator) override;

 private:
  ForceAxisFeedbackControllerMessage force_axis_feedback_params_;

  const franka::Model *model_;

  double translational_stiffness_ = 600;
  double rotational_stiffness_ = 50;
  Eigen::MatrixXd stiffness_;
  Eigen::MatrixXd damping_;

  Eigen::Vector3d axis_;
};

#endif  // FRANKA_INTERFACE_FEEDBACK_CONTROLLER_FORCE_AXIS_IMPEDENCE_FEEDBACK_CONTROLLER_H_