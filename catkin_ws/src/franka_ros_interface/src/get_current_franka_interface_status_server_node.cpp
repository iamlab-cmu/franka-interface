#include <ros/ros.h>
#include "franka_ros_interface/get_current_franka_interface_status_server.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_franka_interface_status_server", ros::init_options::AnonymousName);

  franka_ros_interface::GetCurrentFrankaInterfaceStatusServer get_current_franka_interface_status_service("get_franka_interface_status_server");

  return 0;
}