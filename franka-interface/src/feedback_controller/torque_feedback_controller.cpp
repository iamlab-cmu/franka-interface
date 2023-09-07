//
// Created by Ruthrash Hari on 7/9/23
// 

#include <exception>
#include "franka-interface/feedback_controller/torque_feedback_controller.h"


void TorqueFeedbackController::parse_sensor_data(const franka::RobotState &robot_state){
    tau_d.resize(7);
    SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readFeedbackControllerSensorMessage(torque_feedback_sensor_msg_);
    if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
        for(int i = 0; i < 7 ; i++){
            tau_d(i) = torque_feedback_sensor_msg_.joint_torques_cmd(i);
        }
    }
}
void TorqueFeedbackController::get_next_step(const franka::RobotState &robot_state,
                                            TrajectoryGenerator *traj_generator){

  Eigen::VectorXd::Map(&tau_d_array_[0], 7) = tau_d;

}