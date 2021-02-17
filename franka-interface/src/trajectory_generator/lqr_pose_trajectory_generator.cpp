//
// Created by Saumya on 10/03/20.
// 
// Regularization to a desired position. Keeping the orientation same as initial orientation.
// Finding discete-time LQR gains
#include<Eigen/StdVector>
#include "franka-interface/trajectory_generator/lqr_pose_trajectory_generator.h"
#include "franka-interface/franka_robot.h"

void LqrPoseTrajectoryGenerator::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readTrajectoryGeneratorSensorMessage(pose_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
    for (int i = 0; i < 3; i++) {
      desired_position_[i] = pose_sensor_msg_.position(i);
      object_position_[i] = pose_sensor_msg_.position(i);
    }

    desired_orientation_.w() = pose_sensor_msg_.quaternion(0);
    desired_orientation_.x() = pose_sensor_msg_.quaternion(1);
    desired_orientation_.y() = pose_sensor_msg_.quaternion(2);
    desired_orientation_.z() = pose_sensor_msg_.quaternion(3);

    object_position_[3] = pose_sensor_msg_.quaternion(0);
    object_position_[4] = pose_sensor_msg_.quaternion(1);
    object_position_[5] = pose_sensor_msg_.quaternion(2);
    object_position_[6] = pose_sensor_msg_.quaternion(3);
  }
    
}

void LqrPoseTrajectoryGenerator::initialize_model(FrankaRobot *robot){
  model_ = robot->getModel();
  controlt_.setZero();
  rho = Eigen::VectorXd(2000).setZero();
  rho.head(int(2000/2)) << Eigen::Matrix<double, int(2000/2), 1>::Zero();
  rho.tail(int(2000/2)) << rho.head(int(2000/2)).array() + 1;
}

void LqrPoseTrajectoryGenerator::get_next_step(const franka::RobotState &robot_state) {
  const int n = 6;
  const int m = 3;
  const int N_lqr = 2000;
  int N_update_mpc = 1000;

  Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  std::array<double, 42> jacobian_array = model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());

  Eigen::Matrix<double, n, 1> curr_state;
  curr_state.head(3) = position;
  curr_state.tail(3) = (jacobian*dq).head(3);

  Eigen::Matrix<double, n, n> Q1 = 1000*Eigen::Matrix<double, n, n>::Identity(n, n);
  Eigen::Matrix<double, n, n> Q2 = 1500*Eigen::Matrix<double, n, n>::Identity(n, n);
  Q1(2,2) = 2000;
  Q1(0,0) = 2000;

  Eigen::Matrix<double, m, m> R1 = Eigen::Matrix<double, m, m>::Identity(m, m);
  Eigen::Matrix<double, m, m> R2 = 1*Eigen::Matrix<double, m, m>::Identity(m, m);
  Eigen::Matrix<double, n, n> A = Eigen::Matrix<double, n, n>::Identity(n, n);
  A.block(0,3,3,3) << dt_, 0, 0, 0, dt_, 0, 0, 0, dt_;

  Eigen::Matrix<double, n, m> B1 = Eigen::Matrix<double, n, m>::Zero(n, m);
  B1.block(3,0,3,3) << dt_, 0, 0, 0, dt_, 0, 0, 0, dt_;
  Eigen::Matrix<double, n, m> B2 = Eigen::Matrix<double, n, m>::Zero(n, m);
  B2.block(3,0,3,3) << dt_, 0, 0, 0, dt_, 0, 0, 0, dt_;

  xf1 = Eigen::Matrix<double, n, 1>::Zero(n, 1);
  xf2 = Eigen::Matrix<double, n, 1>::Zero(n, 1);
  for (int i = 0; i < 3; i++) {
    xf1(i) = desired_position_[i];
    xf2(i) = desired_position_[i];
  }
  xf2(0) = 0.53401321;
  xf2(1) = -0.29;
  xf2(2) = 0.30053127;
  // xf2(1) = -0.35;


  // int N = int (run_time_/dt_) + 2000;
  K_.resize(N_lqr);

  Eigen::MatrixXd Kt(m,n);
  Eigen::Matrix<double, n, n> Q_;
  Eigen::Matrix<double, m, m> R_;
  Eigen::Matrix<double, n, m> B_;
  Eigen::Matrix<double, n, 1> xf_;
  Eigen::Matrix<double, n, n> Pt;
  Eigen::Matrix<double, n, n> Ptp1;
  Eigen::Matrix<double, n, N_lqr+1> x = Eigen::Matrix<double, n, N_lqr+1>::Zero(n, N_lqr+1);
  Eigen::Matrix<double, n, N_lqr+1> xold = Eigen::Matrix<double, n, N_lqr+1>::Zero(n, N_lqr+1);
  Eigen::Matrix<double, m, N_lqr> u = Eigen::Matrix<double, m, N_lqr>::Zero(m, N_lqr);
  Eigen::Matrix<double, m, N_lqr> uold = Eigen::Matrix<double, m, N_lqr>::Zero(m, N_lqr);
  // Eigen::MatrixXd x;
  // Eigen::MatrixXd xold;
  // Eigen::MatrixXd u;
  // Eigen::MatrixXd uold;
  
  float tol = 100;
  float tol_old = 100;
  float thresh = 0.1;
  int i=0;
  int n_iter = 5;
  
  bool run_mpc = false;
  bool run_eMosaic = true;

  if(run_eMosaic){
    lqrt_ = 0.0;
    if ( curr_state(1,0) > -0.03){

      rho[0] = 0;
    }
    else {
      rho[0] = 1;
    }
  }

  if ( ( int(time_/dt_) % N_update_mpc ) == 0 && run_mpc){
    
    // iLQR
    std::cout << "Entering iLQR loop at time " << time_ << "\n";
    rho.head(int(N_lqr/2)) << Eigen::Matrix<double, int(N_lqr/2), 1>::Zero();
    rho.tail(int(N_lqr/2)) << rho.head(int(N_lqr/2)).array() + 1;

    lqrt_ = 0.0;
    while ((thresh>1e-6) && (i < n_iter)){
      if (i > 0){
        tol_old = tol + 0;
        xold = 1*x;
        uold = 1*u;
      }
      // LQR BEGIN
      // backward ricatti
      Ptp1 = 1*Q2;
      for (int t = N_lqr-1; t >=0 ; t--) {
        if (rho[t] == 0){
          B_ = 1*B1;
          Q_ = 1*Q1;
          R_ = 1*R1;
        }
        else{
          B_ = 1*B2;
          Q_ = 1*Q2;
          R_ = 1*R2;
        }
        Kt = ( -(R_ + B_.transpose()*Ptp1*B_).inverse() )* B_.transpose()*Ptp1*A;
        Pt = Q_ + A.transpose()*( Eigen::MatrixXd::Identity(n, n) + Ptp1*B_*R_.inverse()*B_.transpose() ).inverse()*Ptp1*A;
        Ptp1 = 1*Pt;
        K_[t] = Kt;
      } //end backward ricatti for loop
      // std::cout << "Backprop done " << time_ << "\n";
      // Forward propagation
      x.col(0) = curr_state;
      for (int t = 0; t < N_lqr ; t++) {
        if (rho[t] == 0){
          xf_ = 1*xf1;
          B_ = 1*B1;
        }
        else{
          xf_ = 1*xf2;
          B_ = 1*B2;
        }
        u.col(t) = K_[t]*(x.col(t) - xf_);
        x.col(t+1) = A*x.col(t) + B_*u.col(t);
      }//end Forward propagation for loop
      // LQR end

      //Update rho
      for (int t = 0; t < N_lqr ; t++) {
        // if ( sqrt( square( (x.block(0,t, 3,1)-xf1).array() ).sum() )> 0.06){
        // if ( abs(x(1,t)-xf1(1))> 0.01){
        if ( x(1,t) > -0.03){

          rho[t] = 0;
        }
        else {
          rho[t] = 1;
        }
      }
      if (i > 0){
        tol = square( (x-xold).array()).sum() + square((u-uold).array()).sum();
        thresh = (tol-tol_old)*(tol-tol_old);
      }
      i = i+1;
      
    }//end while loop
    std::cout<< "rho lqr: " << rho << "\n";
    std::cout<< "Iterations: " << i << "\n";
  // controlt_ = u.col(0);
  } // end of if statements
    

  // Ptp1 = 1*Q;
  // for (int i = N-1; i >=0 ; i--) {
  //   Kt = ( -(R + B.transpose()*Ptp1*B).inverse() )* B.transpose()*Ptp1*A;
  //   Pt = Q + A.transpose()*( Eigen::MatrixXd::Identity(n, n) + Ptp1*B*R.inverse()*B.transpose() ).inverse()*Ptp1*A;
  //   Ptp1 = 1*Pt;
  //   K_[i] = Kt;
  // }

}
  