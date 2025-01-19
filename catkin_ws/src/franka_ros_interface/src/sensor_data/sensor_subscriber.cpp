#include <ros/ros.h>

#include "franka_ros_interface/sensor_data/sensor_subscriber_handler.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sensor_subscriber_node", ros::init_options::AnonymousName);

  ros::NodeHandle n;
  franka_ros_interface::SensorSubscriberHandler handler(n);
  std::string robot_num;
  if (n.getParam("robot_num", robot_num))
  {
    if (robot_num == "1")
    {
      ros::Subscriber sub = n.subscribe("franka_ros_interface/sensor", 10, &franka_ros_interface::SensorSubscriberHandler::SensorSubscriberCallback, &handler);
    }
    else
    {
      ros::Subscriber sub = n.subscribe("franka_ros_interface_"+robot_num+"/sensor", 10, &franka_ros_interface::SensorSubscriberHandler::SensorSubscriberCallback, &handler);
    }
  }
  else
  {
    ros::Subscriber sub = n.subscribe("franka_ros_interface/sensor", 10, &franka_ros_interface::SensorSubscriberHandler::SensorSubscriberCallback, &handler);
  }

  ros::spin();

  return 0;
}
