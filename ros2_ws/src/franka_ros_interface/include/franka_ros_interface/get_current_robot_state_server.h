#ifndef FRANKA_ROS_INTERFACE_GET_CURRENT_ROBOT_STATE_SERVER_H
#define FRANKA_ROS_INTERFACE_GET_CURRENT_ROBOT_STATE_SERVER_H

#include <iostream>
#include <thread>
#include <mutex>
#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "franka_interface_msgs/srv/get_current_robot_state.hpp"
#include "franka_interface_msgs/msg/robot_state.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace franka_ros_interface  
{ 
  class GetCurrentRobotStateServer : public rclcpp::Node
  {
    protected:

      rclcpp::Service<franka_interface_msgs::srv::GetCurrentRobotState>::SharedPtr get_current_robot_state_server_;
      rclcpp::Subscription<franka_interface_msgs::msg::RobotState>::SharedPtr robot_state_subscriber_;
      std::string robot_state_topic_name_;
      std::mutex current_robot_state_mutex_;
      franka_interface_msgs::msg::RobotState current_robot_state_;

    public:

      GetCurrentRobotStateServer();

      ~GetCurrentRobotStateServer(void){}

    private: 

      void robot_state_subscriber_callback(const franka_interface_msgs::msg::RobotState::SharedPtr robot_state);
      bool get_current_robot_state_service(const std::shared_ptr<franka_interface_msgs::srv::GetCurrentRobotState::Request> req,
                                           std::shared_ptr<franka_interface_msgs::srv::GetCurrentRobotState::Response> res);
  };
}

#endif // FRANKA_ROS_INTERFACE_GET_CURRENT_ROBOT_STATE_SERVER_H