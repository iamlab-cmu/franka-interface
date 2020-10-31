#include "franka_ros_interface/sensor_data/sensor_subscriber_handler.h"

namespace franka_ros_interface
{
  SensorSubscriberHandler::SensorSubscriberHandler(ros::NodeHandle& nh)
  : shared_memory_handler_  ()
  , nh_                     (nh) {
    ROS_INFO("created_sensor_subscriber_handler");
  }

  void SensorSubscriberHandler::SensorSubscriberCallback(const franka_interface_msgs::SensorDataGroup::ConstPtr& sensor_group_msg) {
    ROS_INFO("Got sensor message! TG %s | FC %s | TH %s", 
      sensor_group_msg->has_trajectory_generator_sensor_data ? "yes" : "no",
      sensor_group_msg->has_feedback_controller_sensor_data ? "yes" : "no",
      sensor_group_msg->has_termination_handler_sensor_data ? "yes" : "no"
    );

    shared_memory_handler_.tryToLoadSensorDataGroupIntoSharedMemory(sensor_group_msg);
  }
}
