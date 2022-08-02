#ifndef FRANKA_ROS_INTERFACE_SENSOR_SUBSCRIBER_HANDLER_H
#define FRANKA_ROS_INTERFACE_SENSOR_SUBSCRIBER_HANDLER_H

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "franka_interface_msgs/msg/sensor_data_group.hpp"
#include "franka_ros_interface/shared_memory_handler.h"

using std::placeholders::_1;
namespace franka_ros_interface  
{ 
  class SensorDataSubscriber : public rclcpp::Node
  {
    protected:

      SharedMemoryHandler shared_memory_handler_;

    public:

      SensorDataSubscriber();
      ~SensorDataSubscriber(){};
      
    private:
      rclcpp::Subscription<franka_interface_msgs::msg::SensorDataGroup>::SharedPtr sensor_data_subscriber_;

      void sensor_data_callback(const franka_interface_msgs::msg::SensorDataGroup::SharedPtr sensor_group_msg);
  };
}

#endif // 