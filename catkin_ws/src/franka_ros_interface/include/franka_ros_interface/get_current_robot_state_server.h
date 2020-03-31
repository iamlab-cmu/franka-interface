#ifndef FRANKA_ROS_INTERFACE_GET_CURRENT_ROBOT_STATE_SERVER_H
#define FRANKA_ROS_INTERFACE_GET_CURRENT_ROBOT_STATE_SERVER_H

#include <iostream>
#include <thread>
#include <mutex>
#include <ros/ros.h>

#include "franka_interface_msgs/GetCurrentRobotStateCmd.h"
#include "franka_interface_msgs/RobotState.h"

namespace franka_ros_interface  
{ 
  class GetCurrentRobotStateServer
  {
    protected:

      ros::NodeHandle nh_;
      ros::ServiceServer server; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
      std::string robot_state_topic_name_;
      static std::mutex current_robot_state_mutex_;
      static franka_interface_msgs::RobotState current_robot_state_;

    public:

      GetCurrentRobotStateServer(std::string name);

      ~GetCurrentRobotStateServer(void){}

      static bool get_current_robot_state(franka_interface_msgs::GetCurrentRobotStateCmd::Request &req, franka_interface_msgs::GetCurrentRobotStateCmd::Response &res);
      static void robot_state_sub_cb(const franka_interface_msgs::RobotState& robot_state);

  };
}

#endif // FRANKA_ROS_INTERFACE_GET_CURRENT_ROBOT_STATE_SERVER_H