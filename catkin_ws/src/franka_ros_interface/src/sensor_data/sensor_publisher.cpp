//no longer need sensor publisher on cpp side - now using publisher on python side


#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include "franka_interface_msgs/SensorData.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_sensor_publisher");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<franka_interface_msgs::SensorData>("dummy_time", 1000);

  ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {

   franka_interface_msgs::SensorData f_data;


    f_data.sensorDataInfo = "Torque";
    f_data.size =10;
    f_data.sensorData = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

      ROS_INFO("%f", f_data.sensorData[0]);

    chatter_pub.publish(f_data);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}