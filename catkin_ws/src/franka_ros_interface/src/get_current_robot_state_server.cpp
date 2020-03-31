#include <ros/ros.h>
#include "franka_ros_interface/get_current_robot_state_server.h"

namespace franka_ros_interface
{
  std::mutex GetCurrentRobotStateServer::current_robot_state_mutex_;
  franka_interface_msgs::RobotState GetCurrentRobotStateServer::current_robot_state_;

  GetCurrentRobotStateServer::GetCurrentRobotStateServer(std::string name) :  nh_("~")
  {
    nh_.param("robot_state_topic_name", robot_state_topic_name_, std::string("/robot_state_publisher_node/robot_state"));

    ros::Subscriber sub = nh_.subscribe(robot_state_topic_name_, 10, robot_state_sub_cb);
    ros::ServiceServer service = nh_.advertiseService("get_current_robot_state_server", get_current_robot_state);
    ROS_INFO("Get Current Robot State Server Started");
    ros::spin();
  }

  void GetCurrentRobotStateServer::robot_state_sub_cb(const franka_interface_msgs::RobotState& robot_state)
  {
    if (current_robot_state_mutex_.try_lock()) {
      current_robot_state_ = robot_state;
      current_robot_state_mutex_.unlock();
    }
  }

  bool GetCurrentRobotStateServer::get_current_robot_state(franka_interface_msgs::GetCurrentRobotStateCmd::Request &req, 
                                                           franka_interface_msgs::GetCurrentRobotStateCmd::Response &res)
  {
    ROS_DEBUG("Get Current Robot State Server request received.");
    current_robot_state_mutex_.lock();
    res.robot_state = current_robot_state_;
    current_robot_state_mutex_.unlock();
    ROS_DEBUG("Get Current Robot State Server request processed.");
    
    return true;
  }
}
