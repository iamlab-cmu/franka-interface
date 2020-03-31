#include <ros/ros.h>
#include <ros/console.h>
#include "franka_ros_interface/execute_skill_action_server.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "execute_skill_action_server_node", ros::init_options::AnonymousName);

  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  franka_ros_interface::ExecuteSkillActionServer execute_skill_action_server("execute_skill");
  ros::spin();

  return 0;
}