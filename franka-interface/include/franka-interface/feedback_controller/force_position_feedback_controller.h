#ifndef FRANKA_INTERFACE_FEEDBACK_CONTROLLER_FORCE_POSITION_FEEDBACK_CONTROLLER_H_
#define FRANKA_INTERFACE_FEEDBACK_CONTROLLER_FORCE_POSITION_FEEDBACK_CONTROLLER_H_

#include <Eigen/Dense>

#include "franka-interface/feedback_controller/feedback_controller.h"

// A Hybrid Force Position Controller
class ForcePositionFeedbackController : public FeedbackController {
 public:
  using FeedbackController::FeedbackController;

  void parse_parameters() override;

  void initialize_controller(FrankaRobot *robot) override;

  void get_next_step(const franka::RobotState &robot_state, 
                     TrajectoryGenerator *traj_generator) override;

  void parse_sensor_data(const franka::RobotState &robot_state) override;

 private:
  ForcePositionFeedbackControllerMessage force_position_feedback_params_;
  ForcePositionControllerSensorMessage force_position_sensor_msg_;
  const franka::Model *model_;

  Eigen::Matrix<double, 6, 6> position_kps_ = Eigen::MatrixXd::Zero(6, 6);
  Eigen::Matrix<double, 6, 6> position_kds_ = Eigen::MatrixXd::Zero(6, 6);
  Eigen::Matrix<double, 6, 6> force_kps_ = Eigen::MatrixXd::Zero(6, 6);
  Eigen::Matrix<double, 6, 6> force_kis_ = Eigen::MatrixXd::Zero(6, 6);

  Eigen::Matrix<double, 6, 6> S_ = Eigen::MatrixXd::Identity(6, 6);
  Eigen::Matrix<double, 6, 6> Sp_= Eigen::MatrixXd::Zero(6, 6);

  Eigen::Matrix<double, 6, 1> last_xes_ = Eigen::MatrixXd::Zero(6, 1);
  Eigen::Matrix<double, 6, 1> total_fes_ = Eigen::MatrixXd::Zero(6, 1);

  Eigen::Matrix<double, 6, 1> xe_, fe_, xes_, fes_;
  Eigen::Matrix<double, 7, 1> tau_x_, tau_f_, tau_task_, tau_d_;

};

#endif  // FRANKA_INTERFACE_FEEDBACK_CONTROLLER_FORCE_POSITION_FEEDBACK_CONTROLLER_H_