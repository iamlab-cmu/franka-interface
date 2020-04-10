#ifndef FRANKA_INTERFACE_FEEDBACK_CONTROLLER_EE_CARTESIAN_IMPEDANCE_FEEDBACK_CONTROLLER_H_
#define FRANKA_INTERFACE_FEEDBACK_CONTROLLER_EE_CARTESIAN_IMPEDANCE_FEEDBACK_CONTROLLER_H_

#include <Eigen/Dense>

#include "franka-interface/feedback_controller/cartesian_impedance_feedback_controller.h"

class EECartesianImpedanceFeedbackController : public CartesianImpedanceFeedbackController {
 public:
  using CartesianImpedanceFeedbackController::CartesianImpedanceFeedbackController;
  
  void initialize_controller(FrankaRobot *robot) override;

  void get_next_step(const franka::RobotState &robot_state, 
                     TrajectoryGenerator *traj_generator) override;

 private:
  Eigen::Matrix<double, 3, 3> rot_stiffness_ = Eigen::MatrixXd::Identity(3, 3);
  Eigen::Matrix<double, 3, 3> transl_stiffness_ = Eigen::MatrixXd::Identity(3, 3);
  Eigen::Matrix<double, 3, 3> rot_damping_ = Eigen::MatrixXd::Identity(3, 3);
  Eigen::Matrix<double, 3, 3> transl_damping_ = Eigen::MatrixXd::Identity(3, 3);
};

#endif  // FRANKA_INTERFACE_FEEDBACK_CONTROLLER_EE_CARTESIAN_IMPEDANCE_FEEDBACK_CONTROLLER_H_