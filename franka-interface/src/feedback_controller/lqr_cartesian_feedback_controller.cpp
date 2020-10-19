//
// Created by Saumya on 10/04/20.
//
#include "franka-interface/feedback_controller/lqr_cartesian_feedback_controller.h"

#include <exception>
#include <franka/rate_limiting.h>
#include "franka-interface/trajectory_generator/lqr_pose_trajectory_generator.h"

void LqrCartesianFeedbackController::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = lqr_cartesian_feedback_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){

    if(lqr_cartesian_feedback_params_.translational_stiffnesses_size() == 3){
      for(size_t i = 0; i < 3; i++) {
        translational_stiffnesses_[i] = lqr_cartesian_feedback_params_.translational_stiffnesses(i);
      }
    }

    if(lqr_cartesian_feedback_params_.rotational_stiffnesses_size() == 3){
      for(size_t i = 0; i < 3; i++) {
        rotational_stiffnesses_[i] = lqr_cartesian_feedback_params_.rotational_stiffnesses(i);
      }
    }
  } else {
    std::cout << "Parsing LqrCartesianFeedbackController params failed. Data size = " << data_size << std::endl;
  }
}

void LqrCartesianFeedbackController::initialize_controller(FrankaRobot *robot) {
  model_ = robot->getModel();

  stiffness_ = Eigen::MatrixXd(6,6);
  stiffness_.setZero();
  damping_ = Eigen::MatrixXd(6,6);
  damping_.setZero();

  for (int i = 0; i < 3; i++) {
    stiffness_(i, i) = translational_stiffnesses_[i];
    stiffness_(i + 3, i + 3) = rotational_stiffnesses_[i];
  }
  for (int i = 0; i < 6; i++) {
    damping_(i, i) = 2. * sqrt(stiffness_(i, i));
  }
  jacobian_prev_ = Eigen::MatrixXd(6,7);
  jacobian_prev_.setZero();
  f_task_ = Eigen::VectorXd(6);
  f_task_.setZero();
}

void LqrCartesianFeedbackController::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readFeedbackControllerSensorMessage(lqr_cartesian_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
    for (int i = 0; i < 3; i++) {
      stiffness_(i, i) = lqr_cartesian_sensor_msg_.translational_stiffnesses(i);
      stiffness_(i + 3, i + 3) = lqr_cartesian_sensor_msg_.rotational_stiffnesses(i);
    }
    for (int i = 0; i < 6; i++) {
      damping_(i, i) = 2. * sqrt(stiffness_(i, i));
    }
  }
}

void LqrCartesianFeedbackController::get_next_step(const franka::RobotState &robot_state,
                                                         TrajectoryGenerator *traj_generator) {
  std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
//   std::array<double, 7> gravity_array = model_->gravity(robot_state);
  std::array<double, 49> mass_array = model_->mass(robot_state);
  std::array<double, 42> jacobian_array = model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);

  // convert to Eigen
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
//   Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 7> > mass(mass_array.data());
  Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  LqrPoseTrajectoryGenerator* pose_trajectory_generator = dynamic_cast<LqrPoseTrajectoryGenerator*>(traj_generator);

  if (pose_trajectory_generator == nullptr) {
    throw std::bad_cast();
  }

  Eigen::Vector3d goal_position(pose_trajectory_generator->get_goal_position());
  Eigen::Quaterniond goal_orientation(pose_trajectory_generator->get_goal_orientation());

  // compute error to desired equilibrium pose
  // position error
  Eigen::Matrix<double, 6, 1> pos_error = Eigen::Matrix<double, 6, 1>::Zero(6, 1);
  pos_error.head(3) << position - goal_position;
  // Velocity error
  pos_error.tail(3) << (jacobian * dq).head(3);

  
  // orientation error
  // Eigen::Matrix<double, 6, 1> ori_error = Eigen::Matrix<double, 6, 1>::Zero(6, 1);
  // Eigen::Matrix<double, 6, 1> ori_error_pos = Eigen::Matrix<double, 6, 1>::Zero(6, 1);
  // Eigen::Matrix<double, 6, 1> ori_error_vel = Eigen::Matrix<double, 6, 1>::Zero(6, 1);
  // "difference" quaternion
  if (goal_orientation.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation * goal_orientation.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // // compute "orientation error"
  // ori_error_pos.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
  // ori_error.head(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
  // // Velocity error
  // ori_error_vel.tail(3) << (jacobian * dq).tail(3);
  // ori_error.tail(3) << (jacobian * dq).tail(3);

  // // compute control
  // Eigen::Matrix<double, 6, 1> F = Eigen::Matrix<double, 6, 1>::Zero(6, 1);
  // F.head(3) << 1*pose_trajectory_generator->K_[int(time_/dt_)]*pos_error; //LQR
  // // F.tail(3) << 1*pose_trajectory_generator->K_[int(time_/dt_)]*ori_error; //LQR

  // Eigen::VectorXd tau_task_pos(7), tau_task_ori(7), tau_d(7);
  // tau_task_pos << mass*jacobian.transpose() * F;
  // tau_task_ori << jacobian.transpose() * (-stiffness_ * ori_error_pos - damping_ * ori_error_vel); // Impendance control
  // // tau_task << jacobian.transpose()*( jacobian*mass.inverse()*jacobian.transpose() ).inverse()*F;
  // tau_d << tau_task_pos + tau_task_ori + coriolis;

  // Cartesian control
  Eigen::VectorXd tau_task(7), tau_d(7), error(6);

  error.head(3) << position - goal_position;
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
  Eigen::Matrix<double, 6, 6> delta;
  Eigen::Matrix<double, 6, 7> Jdot;
  delta = (jacobian*mass.inverse()*jacobian.transpose()).inverse();
  Jdot = (jacobian - jacobian_prev_)/0.001;
  // f_task_.setZero();
  f_task_ = -stiffness_ * error - damping_ * (jacobian * dq);
  tau_task << jacobian.transpose() * delta * f_task_;
  tau_d << tau_task + coriolis - jacobian.transpose()*delta*Jdot*dq;
  jacobian_prev_ = 1*jacobian;

  Eigen::VectorXd::Map(&tau_d_array_[0], 7) = tau_d;
}