#ifndef FRANKA_ROS_INTERFACE_ROBOT_STATE_PUBLISHER_H
#define FRANKA_ROS_INTERFACE_ROBOT_STATE_PUBLISHER_H

#include <iostream>
#include <thread>
#include <array>
#include <chrono>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <franka_interface_msgs/RobotState.h> // Note: "Action" is appended

#include "franka_ros_interface/shared_memory_handler.h"

namespace franka_ros_interface  
{ 
  class RobotStatePublisher
  {
    protected:

      ros::NodeHandle nh_;
      ros::Publisher robot_state_pub_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
      std::string topic_name_;

      double publish_frequency_;
      franka_ros_interface::SharedMemoryHandler shared_memory_handler_;
      
      bool has_seen_one_robot_state_;
      franka_interface_msgs::RobotState last_robot_state_;
      std::array<double, 144> last_robot_frames_;

      void BroadcastRobotFrames();

      const double finger_offset_ = 0.0584;
      tf::TransformBroadcaster br_;
      const std::vector<std::string> frame_names_ = {"panda_link1", "panda_link2", "panda_link3",
                                                    "panda_link4", "panda_link5", "panda_link6",
                                                    "panda_link7", "panda_link8", "panda_end_effector"};

    public:

      RobotStatePublisher(std::string name);

      ~RobotStatePublisher(void){}

  };
}

#endif // FRANKA_ROS_INTERFACE_ROBOT_STATE_PUBLISHER_H