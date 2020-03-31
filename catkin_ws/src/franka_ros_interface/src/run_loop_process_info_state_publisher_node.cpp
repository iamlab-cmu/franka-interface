#include <ros/ros.h>
#include "franka_ros_interface/run_loop_process_info_state_publisher.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "run_loop_process_info_state_publisher_node", ros::init_options::AnonymousName);

  franka_ros_interface::RunLoopProcessInfoStatePublisher run_loop_process_info_state_publisher("run_loop_process_info_state");

  return 0;
}