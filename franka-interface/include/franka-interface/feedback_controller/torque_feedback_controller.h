#ifndef FRANKA_INTERFACE_FEEDBACK_CONTROLLER_TORQUE_FEEDBACK_CONTROLLER_H_
#define FRANKA_INTERFACE_FEEDBACK_CONTROLLER_TORQUE_FEEDBACK_CONTROLLER_H_


#include <Eigen/Dense>

#include "franka-interface/feedback_controller/feedback_controller.h"

class TorqueFeedbackController : public FeedbackController{
  public:
    using FeedbackController::FeedbackController;
    void parse_parameters() override;    
    void initialize_controller(FrankaRobot *robot) override;
    void parse_sensor_data(const franka::RobotState &robot_state) override;
    void get_next_step(const franka::RobotState &robot_state, 
                        TrajectoryGenerator *traj_generator) override;
  
  protected: 
    TorqueControllerSensorMessage torque_feedback_sensor_msg_;
    Eigen::VectorXd tau_d;    
};

#endif  // FRANKA_INTERFACE_FEEDBACK_CONTROLLER_TORQUE_FEEDBACK_CONTROLLER_H_
