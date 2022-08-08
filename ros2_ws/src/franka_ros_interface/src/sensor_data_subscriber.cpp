#include "franka_ros_interface/sensor_data_subscriber.h"

namespace franka_ros_interface
{
  SensorDataSubscriber::SensorDataSubscriber() : Node("sensor_data_subscriber"),
      shared_memory_handler_loader_("franka_ros_interface", "franka_ros_interface::BaseSharedMemoryHandler")
  {
    shared_memory_handler_ = shared_memory_handler_loader_.createSharedInstance("franka_ros_interface::SharedMemoryHandler");
    this->declare_parameter<std::string>("sensor_data_topic_name", "/sensor_data_1/sensor_data");
    this->get_parameter("sensor_data_topic_name", sensor_data_topic_name_);
    sensor_data_subscriber_ = this->create_subscription<franka_interface_msgs::msg::SensorDataGroup>(
      sensor_data_topic_name_, 10, std::bind(&SensorDataSubscriber::sensor_data_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Sensor Data Subscriber Started");
  }

  void SensorDataSubscriber::sensor_data_callback(const franka_interface_msgs::msg::SensorDataGroup::SharedPtr sensor_group_msg) {
    RCLCPP_INFO(this->get_logger(), "Got sensor message! TG %s | FC %s | TH %s", 
      sensor_group_msg->has_trajectory_generator_sensor_data ? "yes" : "no",
      sensor_group_msg->has_feedback_controller_sensor_data ? "yes" : "no",
      sensor_group_msg->has_termination_handler_sensor_data ? "yes" : "no"
    );

    shared_memory_handler_->loadSensorDataGroupIntoSharedMemory(sensor_group_msg);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<franka_ros_interface::SensorDataSubscriber>());
  rclcpp::shutdown();
  return 0;
}