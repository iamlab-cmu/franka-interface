#include "franka_ros_interface/franka_interface_status_publisher.h"

namespace franka_ros_interface
{
  FrankaInterfaceStatusPublisher::FrankaInterfaceStatusPublisher(std::string name) :  nh_("~"),
                                                                topic_name_(name)
  {
    nh_.param("publish_frequency", publish_frequency_, (double) 100.0);
    franka_interface_status_pub_ = nh_.advertise<franka_interface_msgs::FrankaInterfaceStatus>(topic_name_, 100);

    shared_memory_handler_ = new franka_ros_interface::SharedMemoryHandler();

    ROS_INFO("FrankaInterface Status Publisher Started");

    ros::Rate loop_rate(publish_frequency_);
    while (ros::ok())
    {
        // get franka_interface
        franka_interface_msgs::FrankaInterfaceStatus franka_interface_status_ = shared_memory_handler_->getFrankaInterfaceStatus();

        if (franka_interface_status_.is_fresh) {
          last_franka_interface_status_ = franka_interface_status_;
          stale_count = 0;
          has_seen_one_franka_interface_status_ = true;
        } else {
          // use previous franka_interface_status if available
          if (has_seen_one_franka_interface_status_) {
            franka_interface_status_ = last_franka_interface_status_;
            stale_count++;
          }
        }

        // only proceed if received at least 1 franka_interface status
        if (has_seen_one_franka_interface_status_) {
          // TODO(jacky): MAGIC NUMBER - Signal not ok if 10 consecutive franka_interface statuses have been stale. Roughly 100 ms
          if (stale_count > 10) {
            franka_interface_status_.is_ready = false;
            franka_interface_status_.is_fresh = false;
          }

          // publish
          franka_interface_status_pub_.publish(franka_interface_status_);

          // increment watchdog counter
          shared_memory_handler_->incrementWatchdogCounter();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
  }
}
