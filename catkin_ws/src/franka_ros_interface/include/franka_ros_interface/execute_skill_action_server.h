#ifndef FRANKA_ROS_INTERFACE_EXECUTE_SKILL_ACTION_SERVER_H
#define FRANKA_ROS_INTERFACE_EXECUTE_SKILL_ACTION_SERVER_H

#include <iostream>
#include <thread>
#include <array>
#include <chrono>
#include <vector>
#include <ros/ros.h>
#include <franka_interface_msgs/ExecuteSkillAction.h> // Note: "Action" is appended
#include <franka_interface_msgs/FrankaInterfaceStatus.h>
#include <actionlib/server/simple_action_server.h>

#include "franka_ros_interface/shared_memory_handler.h"

namespace franka_ros_interface  
{ 
  class ExecuteSkillActionServer
  {
    protected:

      ros::NodeHandle nh_;
      actionlib::SimpleActionServer<franka_interface_msgs::ExecuteSkillAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
      std::string action_name_;

      double publish_frequency_;

      // create messages that are used to published feedback/result
      franka_interface_msgs::ExecuteSkillFeedback feedback_;
      franka_interface_msgs::ExecuteSkillResult result_;

      franka_ros_interface::SharedMemoryHandler shared_memory_handler_;
      franka_interface_msgs::FrankaInterfaceStatus franka_interface_status_;

    public:

      ExecuteSkillActionServer(std::string name);

      ~ExecuteSkillActionServer(void){}

      void executeCB(const franka_interface_msgs::ExecuteSkillGoalConstPtr &goal);

  };
}

#endif // FRANKA_ROS_INTERFACE_EXECUTE_SKILL_ACTION_SERVER_H