#ifndef FRANKA_INTERFACE_FEEDBACK_CONTROLLER_PASSTHROUGH_JOINT_TORQUE_FEEDBACK_CONTROLLER_H_
#define FRANKA_INTERFACE_FEEDBACK_CONTROLLER_PASSTHROUGH_JOINT_TORQUE_FEEDBACK_CONTROLLER_H_

#include "franka-interface/feedback_controller/feedback_controller.h"

// A passthrough feedback controller that just uses the desired joint 
// torques from the sensor subscriber and passes it to the robot
class PassThroughJointTorqueFeedbackController : public FeedbackController {
 public:
  using FeedbackController::FeedbackController;

  void parse_parameters() override;

  void initialize_controller(FrankaRobot *robot) override;

  void get_next_step(const franka::RobotState &robot_state, 
                     TrajectoryGenerator *traj_generator) override;

  void parse_sensor_data(const franka::RobotState &robot_state) override;

 private:
  JointTorqueFeedbackControllerMessage joint_torque_feedback_params_;
  JointTorqueControllerSensorMessage joint_torque_sensor_msg_;
  const franka::Model *model_;

  std::array<double, 7> S_= {};
  std::array<double, 7> remove_gravity_= {};
  std::array<double, 7> desired_joint_torques_= {};

  // Stiffness
  std::array<double, 7> k_gains_ = {{600.0, 600.0, 600.0, 600.0, 
                                     250.0, 150.0, 50.0}};
  // Damping
  std::array<double, 7> d_gains_ = {{50.0, 50.0, 50.0, 50.0, 
                                     30.0, 25.0, 15.0}};
};

#endif  // FRANKA_INTERFACE_FEEDBACK_CONTROLLER_PASSTHROUGH_JOINT_TORQUE_FEEDBACK_CONTROLLER_H_