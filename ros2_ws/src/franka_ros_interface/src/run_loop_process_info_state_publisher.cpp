#include "franka_ros_interface/run_loop_process_info_state_publisher.h"

namespace franka_ros_interface
{
  RunLoopProcessInfoStatePublisher::RunLoopProcessInfoStatePublisher(std::string name) :  Node("run_loop_process_info_state_publisher"),
                                                                                          topic_name_(name)
  {
    run_loop_process_info_state_pub_ = this->create_publisher<franka_interface_msgs::msg::RunLoopProcessInfoState>(topic_name_, 100);
    timer_ = this->create_wall_timer(10ms, std::bind(&RunLoopProcessInfoStatePublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Run Loop Process Info State Publisher Started");
  }

  void RunLoopProcessInfoStatePublisher::timer_callback()
  {
    franka_interface_msgs::msg::RunLoopProcessInfoState run_loop_process_info_state_ = shared_memory_handler_.getRunLoopProcessInfoState();
    run_loop_process_info_state_.header.stamp = this->get_clock()->now();

    if (run_loop_process_info_state_.is_fresh) {
      run_loop_process_info_state_pub_->publish(run_loop_process_info_state_);

      has_seen_one_run_loop_process_info_state_ = true;
      last_run_loop_process_info_state_ = run_loop_process_info_state_;
      last_run_loop_process_info_state_.is_fresh = false;
    } else {
      if (has_seen_one_run_loop_process_info_state_) {
        run_loop_process_info_state_pub_->publish(last_run_loop_process_info_state_);
      }
    }
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<franka_ros_interface::RunLoopProcessInfoStatePublisher>("run_loop_process_info_state"));
  rclcpp::shutdown();

  return 0;
}