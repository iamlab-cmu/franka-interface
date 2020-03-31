#include <ros/ros.h>
#include "franka_ros_interface/franka_interface_status_publisher.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "franka_interface_status_publisher_node", ros::init_options::AnonymousName);

  franka_ros_interface::FrankaInterfaceStatusPublisher franka_interface_status_publisher("franka_interface_status");

  return 0;
}