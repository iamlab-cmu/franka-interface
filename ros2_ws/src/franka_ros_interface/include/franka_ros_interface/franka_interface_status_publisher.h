#ifndef FRANKA_ROS_INTERFACE_ROBOT_STATE_PUBLISHER_H
#define FRANKA_ROS_INTERFACE_ROBOT_STATE_PUBLISHER_H

#include <iostream>
#include <thread>
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <pluginlib/class_loader.hpp>
#include <franka_ros_interface/base_shared_memory_handler.hpp>
#include "franka_interface_msgs/msg/franka_interface_status.hpp"
#include "franka_ros_interface/shared_memory_handler.h"

using namespace std::chrono_literals;
namespace franka_ros_interface  
{ 
  class FrankaInterfaceStatusPublisher : public rclcpp::Node
  {
    protected:

      pluginlib::ClassLoader<franka_ros_interface::BaseSharedMemoryHandler> shared_memory_handler_loader_;
      
      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<franka_interface_msgs::msg::FrankaInterfaceStatus>::SharedPtr franka_interface_status_pub_;
      std::string topic_name_;

      franka_interface_msgs::msg::FrankaInterfaceStatus last_franka_interface_status_;
      bool has_seen_one_franka_interface_status_;
      int stale_count = 0;

      // Signal not ok if 10 consecutive franka_interface statuses have been stale. Roughly 100 ms
      int stale_count_max = 10;

      std::shared_ptr<franka_ros_interface::BaseSharedMemoryHandler> shared_memory_handler_;

    private:
      void timer_callback();

    public:

      FrankaInterfaceStatusPublisher(std::string name);

      ~FrankaInterfaceStatusPublisher(void){}

  };
}

#endif // FRANKA_ROS_INTERFACE_ROBOT_STATE_PUBLISHER_H