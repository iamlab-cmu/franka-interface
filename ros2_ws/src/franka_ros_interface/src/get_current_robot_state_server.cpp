#include "franka_ros_interface/get_current_robot_state_server.h"

namespace franka_ros_interface
{
  GetCurrentRobotStateServer::GetCurrentRobotStateServer() :  Node("get_current_robot_state_server")
  {
    this->declare_parameter<std::string>("robot_state_topic_name", "/robot_state_publisher_node_1/robot_state");
    this->get_parameter("robot_state_topic_name", robot_state_topic_name_);
    robot_state_subscriber_ = this->create_subscription<franka_interface_msgs::msg::RobotState>(
      robot_state_topic_name_, 10, std::bind(&GetCurrentRobotStateServer::robot_state_subscriber_callback, this, _1));
    get_current_robot_state_server_ = this->create_service<franka_interface_msgs::srv::GetCurrentRobotState>("get_current_robot_state_server", 
                                      std::bind(&GetCurrentRobotStateServer::get_current_robot_state_service, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Get Current Robot State Server Started");
  }

  void GetCurrentRobotStateServer::robot_state_subscriber_callback(const franka_interface_msgs::msg::RobotState::SharedPtr robot_state)
  {
    if (current_robot_state_mutex_.try_lock()) {
      current_robot_state_ = *robot_state;
      current_robot_state_mutex_.unlock();
    }
  }

  bool GetCurrentRobotStateServer::get_current_robot_state_service(const std::shared_ptr<franka_interface_msgs::srv::GetCurrentRobotState::Request> req,
                                                                   std::shared_ptr<franka_interface_msgs::srv::GetCurrentRobotState::Response> res)
  {
    RCLCPP_DEBUG(this->get_logger(), "Get Current Robot State Server request received.");
    current_robot_state_mutex_.lock();
    res->robot_state = current_robot_state_;
    current_robot_state_mutex_.unlock();
    RCLCPP_DEBUG(this->get_logger(), "Get Current Robot State Server request processed.");
    
    return true;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<franka_ros_interface::GetCurrentRobotStateServer>());
  rclcpp::shutdown();
}