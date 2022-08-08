#ifndef FRANKA_ROS_INTERFACE_RUN_LOOP_PROCESS_INFO_STATE_PUBLISHER_H
#define FRANKA_ROS_INTERFACE_RUN_LOOP_PROCESS_INFO_STATE_PUBLISHER_H

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
#include <franka_interface_msgs/msg/run_loop_process_info_state.hpp> 

#include "franka_ros_interface/shared_memory_handler.h"

using namespace std::chrono_literals;

namespace franka_ros_interface  
{ 
  class RunLoopProcessInfoStatePublisher : public rclcpp::Node
  {
    protected:

      pluginlib::ClassLoader<franka_ros_interface::BaseSharedMemoryHandler> shared_memory_handler_loader_;
      
      rclcpp::TimerBase::SharedPtr timer_;

      rclcpp::Publisher<franka_interface_msgs::msg::RunLoopProcessInfoState>::SharedPtr  run_loop_process_info_state_pub_;
      std::string topic_name_;

      std::shared_ptr<franka_ros_interface::BaseSharedMemoryHandler> shared_memory_handler_;
      
      bool has_seen_one_run_loop_process_info_state_;
      franka_interface_msgs::msg::RunLoopProcessInfoState last_run_loop_process_info_state_;

    private:
      void timer_callback();

    public:

      RunLoopProcessInfoStatePublisher(std::string name);

      ~RunLoopProcessInfoStatePublisher(void){}

  };
}

#endif // FRANKA_ROS_INTERFACE_RUN_LOOP_PROCESS_INFO_STATE_PUBLISHER_H