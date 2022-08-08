#ifndef FRANKA_ROS_INTERFACE_ROBOT_STATE_PUBLISHER_H
#define FRANKA_ROS_INTERFACE_ROBOT_STATE_PUBLISHER_H

#include <iostream>
#include <thread>
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pluginlib/class_loader.hpp>
#include <franka_ros_interface/base_shared_memory_handler.hpp>
#include <franka_interface_msgs/msg/robot_state.hpp>

#include "franka_ros_interface/shared_memory_handler.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace franka_ros_interface  
{ 
  class RobotStatePublisher : public rclcpp::Node
  {
    protected:

      pluginlib::ClassLoader<franka_ros_interface::BaseSharedMemoryHandler> shared_memory_handler_loader_;
      
      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<franka_interface_msgs::msg::RobotState>::SharedPtr robot_state_pub_;
      std::string topic_name_;

      std::shared_ptr<franka_ros_interface::BaseSharedMemoryHandler> shared_memory_handler_;
      
      bool has_seen_one_robot_state_;
      franka_interface_msgs::msg::RobotState last_robot_state_;
      std::array<double, 144> last_robot_frames_;

      void BroadcastRobotFrames();

      const double finger_offset_ = 0.0584;
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
      const std::vector<std::string> frame_names_ = {"panda_link1", "panda_link2", "panda_link3",
                                                     "panda_link4", "panda_link5", "panda_link6",
                                                     "panda_link7", "panda_link8", "panda_end_effector"};
    private:
      void timer_callback();

    public:

      RobotStatePublisher(std::string name);

      ~RobotStatePublisher(void){}

  };
}

#endif // FRANKA_ROS_INTERFACE_ROBOT_STATE_PUBLISHER_H