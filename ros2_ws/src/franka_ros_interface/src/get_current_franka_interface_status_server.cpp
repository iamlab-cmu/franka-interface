#include "franka_ros_interface/get_current_franka_interface_status_server.h"

namespace franka_ros_interface
{

  GetCurrentFrankaInterfaceStatusServer::GetCurrentFrankaInterfaceStatusServer() :  Node("get_franka_interface_status_server")
  {
    this->declare_parameter<std::string>("franka_interface_status_topic_name", "/franka_interface_status_publisher_node_1/franka_interface_status");
    this->get_parameter("franka_interface_status_topic_name", franka_interface_status_topic_name_);
    franka_interface_status_subscriber_ = this->create_subscription<franka_interface_msgs::msg::FrankaInterfaceStatus>(
      franka_interface_status_topic_name_, 10, std::bind(&GetCurrentFrankaInterfaceStatusServer::franka_interface_status_subscriber_callback, this, _1));
    get_current_franka_interface_status_server_ = this->create_service<franka_interface_msgs::srv::GetCurrentFrankaInterfaceStatus>("~/franka_interface_status", 
                                                  std::bind(&GetCurrentFrankaInterfaceStatusServer::get_current_franka_interface_status_service, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Get Current Franka Interface Status Server Started");
  }

  void GetCurrentFrankaInterfaceStatusServer::franka_interface_status_subscriber_callback(const franka_interface_msgs::msg::FrankaInterfaceStatus::SharedPtr franka_interface_status)
  {
    if (current_franka_interface_status_mutex_.try_lock()) {
      current_franka_interface_status_ = *franka_interface_status;
      current_franka_interface_status_mutex_.unlock();
    }
  }

  bool GetCurrentFrankaInterfaceStatusServer::get_current_franka_interface_status_service(const std::shared_ptr<franka_interface_msgs::srv::GetCurrentFrankaInterfaceStatus::Request> req,
                                                                                          std::shared_ptr<franka_interface_msgs::srv::GetCurrentFrankaInterfaceStatus::Response> res)
  {
    RCLCPP_DEBUG(this->get_logger(), "Get Current Franka Interface Status Server request received.");
    current_franka_interface_status_mutex_.lock();
    res->franka_interface_status = current_franka_interface_status_;
    current_franka_interface_status_mutex_.unlock();
    RCLCPP_DEBUG(this->get_logger(), "Get Current Franka Interface Status Server request processed.");
    
    return true;
  }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<franka_ros_interface::GetCurrentFrankaInterfaceStatusServer>());
  rclcpp::shutdown();
}