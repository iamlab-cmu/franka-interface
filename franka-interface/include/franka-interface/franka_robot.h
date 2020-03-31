#ifndef FRANKA_INTERFACE_ROBOTS_FRANKA_ROBOT_H_
#define FRANKA_INTERFACE_ROBOTS_FRANKA_ROBOT_H_

#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/model.h>


class FrankaRobot
{
 public:

  FrankaRobot(std::string &robot_ip) : 
                                  robot_(robot_ip),
                                  gripper_(robot_ip),
                                  model_(robot_.loadModel()) 
  {};

  franka::Model* getModel()
  {
    return &model_;
  }

  franka::RobotState getRobotState()
  {
    return robot_.readOnce();
  }

  franka::GripperState getGripperState()
  {
    return gripper_.readOnce();
  }

  franka::Robot robot_;
  franka::Gripper gripper_;
  franka::Model model_;

  virtual ~FrankaRobot() = default;

  void automaticErrorRecovery() {
    robot_.automaticErrorRecovery();
  }

  void setJointImpedance(const std::array<double, 7>& K_theta) {
    robot_.setJointImpedance(K_theta);
  }

  void setCartesianImpedance(const std::array<double, 6>& K_x) {
    robot_.setCartesianImpedance(K_x);
  }
};

#endif  // FRANKA_INTERFACE_ROBOTS_FRANKA_ROBOT_H_