#ifndef FRANKA_INTERFACE_FEEDBACK_CONTROLLER_CARTESIAN_IMPEDANCE_FEEDBACK_CONTROLLER_H_
#define FRANKA_INTERFACE_FEEDBACK_CONTROLLER_CARTESIAN_IMPEDANCE_FEEDBACK_CONTROLLER_H_

#include <Eigen/Dense>

#include "franka-interface/feedback_controller/feedback_controller.h"

class CartesianImpedanceFeedbackController : public FeedbackController {
 public:
  using FeedbackController::FeedbackController;

  void parse_parameters() override;

  void initialize_controller(FrankaRobot *robot) override;

  void parse_sensor_data(const franka::RobotState &robot_state) override;
  
  void get_next_step(const franka::RobotState &robot_state, 
                     TrajectoryGenerator *traj_generator) override;

 protected:
  CartesianImpedanceFeedbackControllerMessage cartesian_impedance_feedback_params_;
  CartesianImpedanceSensorMessage cartesian_impedance_sensor_msg_;

  const franka::Model *model_;

  std::array<double, 3> translational_stiffnesses_ = {{600.0, 600.0, 600.0}};
  std::array<double, 3> rotational_stiffnesses_ = {{50.0, 50.0, 50.0}};
  Eigen::MatrixXd stiffness_;
  Eigen::MatrixXd damping_;
};

#endif  // FRANKA_INTERFACE_FEEDBACK_CONTROLLER_CARTESIAN_IMPEDANCE_FEEDBACK_CONTROLLER_H_