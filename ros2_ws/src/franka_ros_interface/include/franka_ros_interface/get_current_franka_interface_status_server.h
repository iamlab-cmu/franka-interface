#ifndef FRANKA_ROS_INTERFACE_GET_CURRENT_FRANKA_INTERFACE_STATUS_SERVER_H
#define FRANKA_ROS_INTERFACE_GET_CURRENT_FRANKA_INTERFACE_STATUS_SERVER_H

#include <iostream>
#include <thread>
#include <mutex>
#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "franka_interface_msgs/srv/get_current_franka_interface_status.hpp"
#include "franka_interface_msgs/msg/franka_interface_status.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace franka_ros_interface  
{ 
  class GetCurrentFrankaInterfaceStatusServer : public rclcpp::Node
  {
    protected:

      rclcpp::Service<franka_interface_msgs::srv::GetCurrentFrankaInterfaceStatus>::SharedPtr get_current_franka_interface_status_server_;
      rclcpp::Subscription<franka_interface_msgs::msg::FrankaInterfaceStatus>::SharedPtr franka_interface_status_subscriber_;
      std::string franka_interface_status_topic_name_;
      std::mutex current_franka_interface_status_mutex_;
      franka_interface_msgs::msg::FrankaInterfaceStatus current_franka_interface_status_;

    public:

      GetCurrentFrankaInterfaceStatusServer();

      ~GetCurrentFrankaInterfaceStatusServer(void){}

    private:

      void franka_interface_status_subscriber_callback(const franka_interface_msgs::msg::FrankaInterfaceStatus::SharedPtr franka_interface_status);
      bool get_current_franka_interface_status_service(const std::shared_ptr<franka_interface_msgs::srv::GetCurrentFrankaInterfaceStatus::Request> req,
                                                       std::shared_ptr<franka_interface_msgs::srv::GetCurrentFrankaInterfaceStatus::Response> res);
  };
}

#endif // FRANKA_ROS_INTERFACE_GET_CURRENT_FRANKA_INTERFACE_STATUS_SERVER_H