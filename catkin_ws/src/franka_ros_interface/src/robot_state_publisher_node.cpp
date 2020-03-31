#include <ros/ros.h>
#include "franka_ros_interface/robot_state_publisher.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_state_publisher_node", ros::init_options::AnonymousName);

  franka_ros_interface::RobotStatePublisher robot_state_publisher("robot_state");

  return 0;
}