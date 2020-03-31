//
// Created by jacky on 2/13/19.
//

#include "franka-interface/feedback_controller/pass_through_feedback_controller.h"

#include "franka-interface/trajectory_generator/impulse_trajectory_generator.h"

void PassThroughFeedbackController::parse_parameters() {
  // pass
}

void PassThroughFeedbackController::initialize_controller(FrankaRobot *robot) {
  model_ = robot->getModel();
}

void PassThroughFeedbackController::get_next_step(const franka::RobotState &robot_state,
                                                  TrajectoryGenerator *traj_generator) {
    
  ImpulseTrajectoryGenerator* impulse_trajectory_generator = dynamic_cast<ImpulseTrajectoryGenerator*>(traj_generator);

  if(impulse_trajectory_generator == nullptr) {
    throw std::bad_cast();
  }

  std::array<double, 6> desired_force_torque_array = impulse_trajectory_generator->get_desired_force_torque();
  Eigen::Map<Eigen::VectorXd> desired_force_torque(desired_force_torque_array.data(), 6);

  // get jacobian
  std::array<double, 42> jacobian_array = model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

  // compute control torques
  Eigen::VectorXd tau_d(7);
  tau_d << jacobian.transpose() * desired_force_torque + coriolis;

  Eigen::VectorXd::Map(&tau_d_array_[0], 7) = tau_d;
}