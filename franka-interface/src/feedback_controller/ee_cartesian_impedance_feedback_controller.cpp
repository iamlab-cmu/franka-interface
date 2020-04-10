//
// Created by mohit on 11/25/18.
//

#include "franka-interface/feedback_controller/ee_cartesian_impedance_feedback_controller.h"

#include <iostream>
#include <exception>

#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"

void EECartesianImpedanceFeedbackController::initialize_controller(FrankaRobot *robot) {
  model_ = robot->getModel();

  for (int i = 0; i < 3; i++) {
    transl_stiffness_(i, i) = translational_stiffnesses_[i];
    rot_stiffness_(i, i) = rotational_stiffnesses_[i];
    transl_damping_(i, i) = 2. * sqrt(translational_stiffnesses_[i]);
    rot_damping_(i, i) = 2. * sqrt(rotational_stiffnesses_[i]);
  }
}

void EECartesianImpedanceFeedbackController::get_next_step(const franka::RobotState &robot_state,
                                                         TrajectoryGenerator *traj_generator) {
  std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
  std::array<double, 42> jacobian_array = model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);

  // convert to Eigen
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
  Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  PoseTrajectoryGenerator* pose_trajectory_generator = dynamic_cast<PoseTrajectoryGenerator*>(traj_generator);

  if(pose_trajectory_generator == nullptr) {
    throw std::bad_cast();
  }

  Eigen::Vector3d position_d(pose_trajectory_generator->get_desired_position());
  Eigen::Quaterniond orientation_d(pose_trajectory_generator->get_desired_orientation());

  // compute error to desired equilibrium pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d;

  // orientation error
  // "difference" quaternion
  if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation * orientation_d.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
  
  Eigen::Matrix3d R = orientation_d.toRotationMatrix();

  //Transform position error
  Eigen::Matrix<double, 3, 1> local_transf;
  Eigen::Matrix<double, 3, 1> local_transf_times_K;
  local_transf= R.transpose()*error.head(3);

  local_transf_times_K= transl_stiffness_*local_transf;

  error.head(3) = -R*local_transf_times_K;        

  //Transform orientation error
  local_transf= R.transpose()*error.tail(3);

  local_transf_times_K= rot_stiffness_*local_transf;
  error.tail(3) = -R*local_transf_times_K;  

  //Transform jacobian * dq to local reference frame 
  Eigen::Matrix<double, 6, 1> Jdq;
  Jdq = jacobian * dq ;

  //Transform position velocities
  local_transf= R.transpose()*Jdq.head(3);
  local_transf_times_K= transl_damping_*local_transf;
  Jdq.head(3) = -R*local_transf_times_K;

  //Transform angular velocities
  local_transf= R.transpose()*Jdq.tail(3);
  local_transf_times_K= rot_damping_*local_transf;
  Jdq.tail(3) = -R*local_transf_times_K;
  
  ////////////////////////////////////////////////////////////////////////////////

  // compute control
  Eigen::VectorXd tau_task(7), tau_d(7);
  
  tau_task << jacobian.transpose() * (error + (Jdq));  
  tau_d << tau_task + coriolis;  

  Eigen::VectorXd::Map(&tau_d_array_[0], 7) = tau_d;
}