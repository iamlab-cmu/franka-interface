#include <ros/ros.h>
#include "franka_ros_interface/get_current_robot_state_server.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_robot_state_server", ros::init_options::AnonymousName);

  franka_ros_interface::GetCurrentRobotStateServer get_current_robot_state_service("get_robot_state_server");

  return 0;
}