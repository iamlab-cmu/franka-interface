#include "franka_ros_interface/get_current_gripper_state_server.h"

namespace franka_ros_interface
{

  GetCurrentGripperStateServer::GetCurrentGripperStateServer() :  Node("get_gripper_state_server")
  {
    this->declare_parameter<std::string>("gripper_state_topic_name", "/gripper_state_publisher_node_1/gripper_state");
    this->get_parameter("gripper_state_topic_name", gripper_state_topic_name_);
    gripper_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      gripper_state_topic_name_, 10, std::bind(&GetCurrentGripperStateServer::gripper_state_subscriber_callback, this, _1));
    get_current_gripper_state_server_ = this->create_service<franka_interface_msgs::srv::GetCurrentGripperState>("get_current_gripper_state_server", 
                                        std::bind(&GetCurrentGripperStateServer::get_current_gripper_state_service, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Get Current Gripper State Server Started");
  }

  void GetCurrentGripperStateServer::gripper_state_subscriber_callback(const sensor_msgs::msg::JointState::SharedPtr gripper_state)
  {
    if (current_gripper_state_mutex_.try_lock()) {
      current_gripper_state_ = *gripper_state;
      current_gripper_state_mutex_.unlock();
    }
  }

  bool GetCurrentGripperStateServer::get_current_gripper_state_service(const std::shared_ptr<franka_interface_msgs::srv::GetCurrentGripperState::Request> req,
                                                                             std::shared_ptr<franka_interface_msgs::srv::GetCurrentGripperState::Response> res)
  {
    RCLCPP_DEBUG(this->get_logger(), "Get Current Gripper State Server request received.");
    current_gripper_state_mutex_.lock();
    res->gripper_state = current_gripper_state_;
    current_gripper_state_mutex_.unlock();
    RCLCPP_DEBUG(this->get_logger(), "Get Current Gripper State Server request processed.");
    
    return true;
  }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<franka_ros_interface::GetCurrentGripperStateServer>());
  rclcpp::shutdown();
}