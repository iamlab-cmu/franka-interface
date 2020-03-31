#include "franka_ros_interface/sensor_data/sensor_subscriber_handler.h"
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

namespace franka_ros_interface
{
  SensorSubscriberHandler::SensorSubscriberHandler(ros::NodeHandle& nh)
  : shared_memory_handler_  ()
  , nh_                     (nh) {
    ROS_INFO("created_sensor_subscriber_handler");
  }

  void SensorSubscriberHandler::SensorSubscriberCallback(const franka_interface_msgs::SensorData::ConstPtr& sensor_msg) {
    ROS_INFO("Got sensor message!");
    ROS_INFO_STREAM(sensor_msg->info);

    shared_memory_handler_.tryToLoadSensorDataIntoSharedMemory(sensor_msg);
  }
}
