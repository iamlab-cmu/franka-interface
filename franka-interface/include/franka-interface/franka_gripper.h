#ifndef FRANKA_INTERFACE_ROBOTS_FRANKA_GRIPPER_H_
#define FRANKA_INTERFACE_ROBOTS_FRANKA_GRIPPER_H_

#include <franka/gripper.h>

class FrankaGripper
{
 public:

  FrankaGripper(std::string &robot_ip) : gripper_(robot_ip) 
  {};

  franka::GripperState getGripperState()
  {
    return gripper_.readOnce();
  }

  franka::Gripper gripper_;

  virtual ~FrankaGripper() = default;
};

#endif  // FRANKA_INTERFACE_ROBOTS_FRANKA_GRIPPER_H_