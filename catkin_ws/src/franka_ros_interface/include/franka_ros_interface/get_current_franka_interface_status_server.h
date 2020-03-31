#ifndef FRANKA_ROS_INTERFACE_GET_CURRENT_FRANKA_INTERFACE_STATUS_SERVER_H
#define FRANKA_ROS_INTERFACE_GET_CURRENT_FRANKA_INTERFACE_STATUS_SERVER_H

#include <iostream>
#include <thread>
#include <mutex>
#include <ros/ros.h>

#include "franka_interface_msgs/GetCurrentFrankaInterfaceStatusCmd.h"
#include "franka_interface_msgs/FrankaInterfaceStatus.h"

#include <boost/circular_buffer.hpp>

namespace franka_ros_interface  
{ 
  class GetCurrentFrankaInterfaceStatusServer
  {
    protected:

      ros::NodeHandle nh_;
      ros::ServiceServer server; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
      std::string franka_interface_status_topic_name_;
      static std::mutex current_franka_interface_status_mutex_;
      static franka_interface_msgs::FrankaInterfaceStatus current_franka_interface_status_;

    public:

      GetCurrentFrankaInterfaceStatusServer(std::string name);

      ~GetCurrentFrankaInterfaceStatusServer(void){}

      static bool get_current_franka_interface_status(franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmd::Request &req, franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmd::Response &res);
      static void franka_interface_status_sub_cb(const franka_interface_msgs::FrankaInterfaceStatus& franka_interface_status);

  };
}

#endif // FRANKA_ROS_INTERFACE_GET_CURRENT_FRANKA_INTERFACE_STATUS_SERVER_H