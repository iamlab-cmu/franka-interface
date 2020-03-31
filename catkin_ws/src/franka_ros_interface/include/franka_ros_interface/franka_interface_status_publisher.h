#ifndef FRANKA_ROS_INTERFACE_ROBOT_STATE_PUBLISHER_H
#define FRANKA_ROS_INTERFACE_ROBOT_STATE_PUBLISHER_H

#include <iostream>
#include <thread>
#include <array>
#include <chrono>
#include <vector>
#include <ros/ros.h>

#include "franka_interface_msgs/FrankaInterfaceStatus.h"
#include "franka_ros_interface/shared_memory_handler.h"

namespace franka_ros_interface  
{ 
  class FrankaInterfaceStatusPublisher
  {
    protected:

      ros::NodeHandle nh_;
      ros::Publisher franka_interface_status_pub_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
      std::string topic_name_;

      franka_interface_msgs::FrankaInterfaceStatus last_franka_interface_status_;
      bool has_seen_one_franka_interface_status_;
      int stale_count = 0;

      double publish_frequency_;
      franka_ros_interface::SharedMemoryHandler *shared_memory_handler_;

    public:

      FrankaInterfaceStatusPublisher(std::string name);

      ~FrankaInterfaceStatusPublisher(void){}

  };
}

#endif // FRANKA_ROS_INTERFACE_ROBOT_STATE_PUBLISHER_H