#ifndef FRANKA_ROS_INTERFACE_SENSOR_SUBSCRIBER_HANDLER_H
#define FRANKA_ROS_INTERFACE_SENSOR_SUBSCRIBER_HANDLER_H

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <franka_ros_interface/base_shared_memory_handler.hpp>
#include "franka_interface_msgs/msg/sensor_data_group.hpp"
#include "franka_ros_interface/shared_memory_handler.h"

using std::placeholders::_1;
namespace franka_ros_interface  
{ 
  class SensorDataSubscriber : public rclcpp::Node
  {
    protected:

      pluginlib::ClassLoader<franka_ros_interface::BaseSharedMemoryHandler> shared_memory_handler_loader_;
      std::shared_ptr<franka_ros_interface::BaseSharedMemoryHandler> shared_memory_handler_;
      std::string sensor_data_topic_name_;

    public:

      SensorDataSubscriber();
      ~SensorDataSubscriber(){};
      
    private:
      rclcpp::Subscription<franka_interface_msgs::msg::SensorDataGroup>::SharedPtr sensor_data_subscriber_;

      void sensor_data_callback(const franka_interface_msgs::msg::SensorDataGroup::SharedPtr sensor_group_msg);
  };
}

#endif // 