#ifndef FRANKA_INTERFACE_FEEDBACK_CONTROLLER_LQR_CARTESIAN_FEEDBACK_CONTROLLER_H_
#define FRANKA_INTERFACE_FEEDBACK_CONTROLLER_LQR_CARTESIAN_FEEDBACK_CONTROLLER_H_

#include <Eigen/Dense>

#include "franka-interface/feedback_controller/feedback_controller.h"

class LqrCartesianFeedbackController : public FeedbackController {
 public:
  using FeedbackController::FeedbackController;

  void parse_parameters() override;

  void initialize_controller(FrankaRobot *robot) override;

  void parse_sensor_data(const franka::RobotState &robot_state) override;
  
  void get_next_step(const franka::RobotState &robot_state, 
                     TrajectoryGenerator *traj_generator) override;

 protected:
  CartesianImpedanceFeedbackControllerMessage lqr_cartesian_feedback_params_;
  CartesianImpedanceSensorMessage lqr_cartesian_sensor_msg_;

  const franka::Model *model_;

  std::array<double, 3> translational_stiffnesses_ = {{600.0, 600.0, 600.0}};
  std::array<double, 3> rotational_stiffnesses_ = {{50.0, 50.0, 50.0}};
  Eigen::MatrixXd stiffness_;
  Eigen::MatrixXd damping_;
  Eigen::MatrixXd jacobian_prev_;
  double cost_= 0.0;
  
  const double  FI_11 = 0.54615;
  const double  FI_12 = 0.87224;
  const double  FI_13 = 0.64068;
  const double  FI_14 = 1.2794;
  const double  FI_15 = 0.83904;
  const double  FI_16 = 0.30301;
  const double  FI_17 = 0.56489;

  const double  FI_21 = 5.1181;
  const double  FI_22 = 9.0657;
  const double  FI_23 = 10.136;
  const double  FI_24 = 5.5903;
  const double  FI_25 = 8.3469;
  const double  FI_26 = 17.133;
  const double  FI_27 = 10.336;

  const double  FI_31 = 0.039533;
  const double  FI_32 = 0.025882;
  const double  FI_33 = -0.04607;
  const double  FI_34 = 0.036194;
  const double  FI_35 = 0.026226;
  const double  FI_36 = -0.021047;
  const double  FI_37 = 0.0035526;

  const double TAU_F_CONST_1 = FI_11/(1+exp(-FI_21*FI_31));
  const double TAU_F_CONST_2 = FI_12/(1+exp(-FI_22*FI_32));
  const double TAU_F_CONST_3 = FI_13/(1+exp(-FI_23*FI_33));
  const double TAU_F_CONST_4 = FI_14/(1+exp(-FI_24*FI_34));
  const double TAU_F_CONST_5 = FI_15/(1+exp(-FI_25*FI_35));
  const double TAU_F_CONST_6 = FI_16/(1+exp(-FI_26*FI_36));
  const double TAU_F_CONST_7 = FI_17/(1+exp(-FI_27*FI_37));
  
};

#endif  // FRANKA_INTERFACE_FEEDBACK_CONTROLLER_LQR_CARTESIAN_FEEDBACK_CONTROLLER_H_