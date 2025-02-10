#include "franka_ros_interface/sensor_data/sensor_subscriber_handler.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sensor_data_subscriber_node", ros::init_options::AnonymousName);

  ros::NodeHandle n("~");
  franka_ros_interface::SensorSubscriberHandler handler(n);
  int robot_num;
  n.param("robot_num", robot_num, 1);
  ros::Subscriber sub;
  if (robot_num == 1)
  {
    sub = n.subscribe("/franka_ros_interface/sensor", 10, &franka_ros_interface::SensorSubscriberHandler::SensorSubscriberCallback, &handler);
  }
  else
  {
    std::string robot_num_string = std::to_string(robot_num);
    sub = n.subscribe("/franka_ros_interface_"+robot_num_string+"/sensor", 10, &franka_ros_interface::SensorSubscriberHandler::SensorSubscriberCallback, &handler);
  }

  ros::spin();

  return 0;
}
