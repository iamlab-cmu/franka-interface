#include <ros/ros.h>
#include "franka_ros_interface/get_current_franka_interface_status_server.h"

namespace franka_ros_interface
{
  std::mutex GetCurrentFrankaInterfaceStatusServer::current_franka_interface_status_mutex_;
  franka_interface_msgs::FrankaInterfaceStatus GetCurrentFrankaInterfaceStatusServer::current_franka_interface_status_;

  GetCurrentFrankaInterfaceStatusServer::GetCurrentFrankaInterfaceStatusServer(std::string name) :  nh_("~")
  {
    nh_.param("franka_interface_status_topic_name", franka_interface_status_topic_name_, std::string("/franka_interface_status_publisher_node/franka_interface_status"));

    ros::Subscriber sub = nh_.subscribe(franka_interface_status_topic_name_, 10, franka_interface_status_sub_cb);
    ros::ServiceServer service = nh_.advertiseService("get_current_franka_interface_status_server", get_current_franka_interface_status);
    ROS_INFO("Get Current FrankaInterface Status Server Started");
    ros::spin();
  }

  void GetCurrentFrankaInterfaceStatusServer::franka_interface_status_sub_cb(const franka_interface_msgs::FrankaInterfaceStatus& franka_interface_status)
  {
    if (current_franka_interface_status_mutex_.try_lock()) {
      current_franka_interface_status_ = franka_interface_status;
      current_franka_interface_status_mutex_.unlock();
    }
  }

  bool GetCurrentFrankaInterfaceStatusServer::get_current_franka_interface_status(franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmd::Request &req, 
                                                                  franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmd::Response &res)
  {
    ROS_DEBUG("Get Current FrankaInterface Status Server request received.");
    current_franka_interface_status_mutex_.lock();
    res.franka_interface_status = current_franka_interface_status_;
    current_franka_interface_status_mutex_.unlock();
    ROS_DEBUG("Get Current FrankaInterface Status Servier request processed.");
    
    return true;
  }
}
