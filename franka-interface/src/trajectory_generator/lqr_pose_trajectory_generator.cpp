//
// Created by Saumya on 10/03/20.
// 
// Regularization to a desired position. Keeping the orientation same as initial orientation.
// Finding discete-time LQR gains
#include<Eigen/StdVector>
#include "franka-interface/trajectory_generator/lqr_pose_trajectory_generator.h"

void LqrPoseTrajectoryGenerator::get_next_step(const franka::RobotState &robot_state) {
  
  // LQR
  const int n = 6;
  const int m = 3;

  Eigen::Matrix<double, n, n> Q = 10000*Eigen::Matrix<double, n, n>::Identity(n, n);
  // Q(1,1) = 10000;
  // Q(4,4) = 50000;
  // Q(1,1) = 100000;
  // Q(1,1) = 100000;
  Eigen::Matrix<double, m, m> R = Eigen::MatrixXd::Identity(m, m);
  Eigen::Matrix<double, n, n> A = Eigen::MatrixXd::Identity(n, n);
  A.block(0,3,3,3) << dt_, 0, 0, 0, dt_, 0, 0, 0, dt_;
  Eigen::Matrix<double, n, m> B = Eigen::MatrixXd::Zero(n, m);
  B.block(3,0,3,3) << dt_, 0, 0, 0, dt_, 0, 0, 0, dt_;

  int N = int (run_time_/dt_) + 2000;
  K_.resize(N);
  // std::cout << "N :" << N << "\n";
  Eigen::MatrixXd Kt(m,n);
  Eigen::Matrix<double, n, n> Pt;
  Eigen::Matrix<double, n, n> Ptp1;
  // std::vector<Eigen::Matrix<double, n, n>, Eigen::aligned_allocator<Eigen::Matrix<double, n, n>> > P(N)
  // Eigen::Map<Eigen::Matrix<double, n, n> > Ptp1(Q.data());
  Ptp1 = 1*Q;
  for (int i = N-1; i >=0 ; i--) {
    // std::cout << "i :" << i << "\n";
    // std::cout << "Ptp1 :" << Ptp1 << "\n";
    Kt = ( -(R + B.transpose()*Ptp1*B).inverse() )* B.transpose()*Ptp1*A;
    Pt = Q + A.transpose()*( Eigen::MatrixXd::Identity(n, n) + Ptp1*B*R.inverse()*B.transpose() ).inverse()*Ptp1*A;
    // Eigen::Map<Eigen::Matrix<double, n, n> > Ptp1(Pt.data());
    Ptp1 = 1*Pt;
    K_[i] = Kt;
    
    // std::cout << "Kt :" << Kt << "\n";
    
  }

}
  