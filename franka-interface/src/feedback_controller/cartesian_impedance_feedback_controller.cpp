//
// Created by mohit on 11/25/18.
//

#include "franka-interface/feedback_controller/cartesian_impedance_feedback_controller.h"

#include <exception>

#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"

void CartesianImpedanceFeedbackController::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = cartesian_impedance_feedback_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){

    if(cartesian_impedance_feedback_params_.translational_stiffnesses_size() == 3){
      for(size_t i = 0; i < 3; i++) {
        translational_stiffnesses_[i] = cartesian_impedance_feedback_params_.translational_stiffnesses(i);
      }
    }

    if(cartesian_impedance_feedback_params_.rotational_stiffnesses_size() == 3){
      for(size_t i = 0; i < 3; i++) {
        rotational_stiffnesses_[i] = cartesian_impedance_feedback_params_.rotational_stiffnesses(i);
      }
    }
  } else {
    std::cout << "Parsing CartesianImpedanceFeedbackController params failed. Data size = " << data_size << std::endl;
  }
}

void CartesianImpedanceFeedbackController::initialize_controller(FrankaRobot *robot) {
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
}

void CartesianImpedanceFeedbackController::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readFeedbackControllerSensorMessage(cartesian_impedance_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
    for (int i = 0; i < 3; i++) {
      stiffness_(i, i) = cartesian_impedance_sensor_msg_.translational_stiffnesses(i);
      stiffness_(i + 3, i + 3) = cartesian_impedance_sensor_msg_.rotational_stiffnesses(i);
    }
    for (int i = 0; i < 6; i++) {
      damping_(i, i) = 2. * sqrt(stiffness_(i, i));
    }
  }
}

void CartesianImpedanceFeedbackController::get_next_step(const franka::RobotState &robot_state,
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

  if (pose_trajectory_generator == nullptr) {
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

  // compute control
  Eigen::VectorXd tau_task(7), tau_d(7);

  // Spring damper system with damping ratio=1
  tau_task << jacobian.transpose() * (-stiffness_ * error - damping_ * (jacobian * dq));
  tau_d << tau_task + coriolis;

  Eigen::VectorXd::Map(&tau_d_array_[0], 7) = tau_d;
}