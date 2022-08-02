#include "franka_ros_interface/sensor_data_subscriber.h"

namespace franka_ros_interface
{
  SensorDataSubscriber::SensorDataSubscriber() : Node("sensor_data_subscriber")
  {
    sensor_data_subscriber_ = this->create_subscription<franka_interface_msgs::msg::SensorDataGroup>(
      "franka_ros_interface/sensor", 10, std::bind(&SensorDataSubscriber::sensor_data_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Sensor Data Subscriber Started");
  }

  void SensorDataSubscriber::sensor_data_callback(const franka_interface_msgs::msg::SensorDataGroup::SharedPtr sensor_group_msg) {
    RCLCPP_INFO(this->get_logger(), "Got sensor message! TG %s | FC %s | TH %s", 
      sensor_group_msg->has_trajectory_generator_sensor_data ? "yes" : "no",
      sensor_group_msg->has_feedback_controller_sensor_data ? "yes" : "no",
      sensor_group_msg->has_termination_handler_sensor_data ? "yes" : "no"
    );

    shared_memory_handler_.loadSensorDataGroupIntoSharedMemory(sensor_group_msg);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<franka_ros_interface::SensorDataSubscriber>());
  rclcpp::shutdown();
  return 0;
}