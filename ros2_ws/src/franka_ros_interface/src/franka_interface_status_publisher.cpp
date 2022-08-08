#include "franka_ros_interface/franka_interface_status_publisher.h"

namespace franka_ros_interface
{
  FrankaInterfaceStatusPublisher::FrankaInterfaceStatusPublisher() :  Node("franka_interface_status_publisher")
  {
    franka_interface_status_pub_ = this->create_publisher<franka_interface_msgs::msg::FrankaInterfaceStatus>("~/franka_interface_status", 100);
    timer_ = this->create_wall_timer(10ms, std::bind(&FrankaInterfaceStatusPublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Franka Interface Status Publisher Started");
  }

  void FrankaInterfaceStatusPublisher::timer_callback()
  {
    franka_interface_msgs::msg::FrankaInterfaceStatus franka_interface_status_ = shared_memory_handler_.getFrankaInterfaceStatus();
    franka_interface_status_.header.stamp = this->get_clock()->now();

    if (franka_interface_status_.is_fresh) {
      last_franka_interface_status_ = franka_interface_status_;
      stale_count = 0;
      has_seen_one_franka_interface_status_ = true;
    } else {
      // use previous franka_interface_status if available
      if (has_seen_one_franka_interface_status_) {
        franka_interface_status_ = last_franka_interface_status_;
        stale_count++;
      }
    }

    // only proceed if received at least 1 franka_interface status
    if (has_seen_one_franka_interface_status_) {
      if (stale_count > stale_count_max) {
        franka_interface_status_.is_ready = false;
        franka_interface_status_.is_fresh = false;
      }

      // publish
      franka_interface_status_pub_->publish(franka_interface_status_);

      // increment watchdog counter
      shared_memory_handler_.incrementWatchdogCounter();
    }
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<franka_ros_interface::FrankaInterfaceStatusPublisher>());
  rclcpp::shutdown();

  return 0;
}