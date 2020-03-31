#ifndef FRANKA_ROS_INTERFACE_SENSOR_SUBSCRIBER_HANDLER_H
#define FRANKA_ROS_INTERFACE_SENSOR_SUBSCRIBER_HANDLER_H

#include <iostream>
#include <thread>
#include <array>
#include <chrono>
#include <vector>
#include <ros/ros.h>
//#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include "franka_interface_msgs/SensorData.h"

#include "franka_interface_msgs/FrankaInterfaceStatus.h"
#include "franka_ros_interface/shared_memory_handler.h"

namespace franka_ros_interface  
{ 
  class SensorSubscriberHandler
  {
    protected:

      ros::NodeHandle nh_;

      double write_frequency_;
      SharedMemoryHandler shared_memory_handler_;

    public:

      SensorSubscriberHandler(ros::NodeHandle& nh);
      ~SensorSubscriberHandler(){};
      
      void SensorSubscriberCallback(const franka_interface_msgs::SensorData::ConstPtr& sensor_msg);
  };
}

#endif // 