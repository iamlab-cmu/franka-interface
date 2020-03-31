#include "franka_ros_interface/robot_state_publisher.h"

namespace franka_ros_interface
{
  RobotStatePublisher::RobotStatePublisher(std::string name) :  nh_("~"),
                                                                topic_name_(name)
  {
    nh_.param("publish_frequency", publish_frequency_, (double) 100.0);
    robot_state_pub_ = nh_.advertise<franka_interface_msgs::RobotState>(topic_name_, 100);

    ROS_INFO("Robot State Publisher Started");

    ros::Rate loop_rate(publish_frequency_);
    while (ros::ok())
    {
        franka_interface_msgs::RobotState robot_state_ = shared_memory_handler_.getRobotState(last_robot_frames_);

        if (robot_state_.is_fresh) {
          robot_state_pub_.publish(robot_state_);

          has_seen_one_robot_state_ = true;
          last_robot_state_ = robot_state_;
          last_robot_state_.is_fresh = false;

          BroadcastRobotFrames();
        } else {
          if (has_seen_one_robot_state_) {
            robot_state_pub_.publish(last_robot_state_);
          }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
  }

  void RobotStatePublisher::BroadcastRobotFrames()
  {
    int offset;
    tf::Transform transform;
    Eigen::Matrix3d R;

    for (int frame = 0; frame < frame_names_.size(); frame++)
    {
      offset = frame * 16;
      transform.setOrigin(tf::Vector3(last_robot_frames_[offset + 12], last_robot_frames_[offset + 13], last_robot_frames_[offset + 14]));
      
      R << last_robot_frames_[offset + 0], last_robot_frames_[offset + 4], last_robot_frames_[offset + 8],
            last_robot_frames_[offset + 1], last_robot_frames_[offset + 5], last_robot_frames_[offset + 9], 
            last_robot_frames_[offset + 2], last_robot_frames_[offset + 6], last_robot_frames_[offset + 10];
      Eigen::Quaterniond eigen_q(R);

      tf::Quaternion q(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w());
      transform.setRotation(q);

      br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "panda_link0", frame_names_[frame]));
    }
    
    // panda_hand position from kFlange
    offset = 7 * 16;

    transform.setOrigin(tf::Vector3(last_robot_frames_[offset + 12], last_robot_frames_[offset + 13], last_robot_frames_[offset + 14]));

    // panda_hand orientation from kEndEffector
    offset = 8 * 16;
    R << last_robot_frames_[offset + 0], last_robot_frames_[offset + 4], last_robot_frames_[offset + 8],
          last_robot_frames_[offset + 1], last_robot_frames_[offset + 5], last_robot_frames_[offset + 9], 
          last_robot_frames_[offset + 2], last_robot_frames_[offset + 6], last_robot_frames_[offset + 10];
    Eigen::Quaterniond eigen_q(R);

    tf::Quaternion q(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w());
    transform.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "panda_link0", "panda_hand"));

    // For grippers
    offset = 7 * 16;
    double gripper_width = last_robot_state_.gripper_width/2.;

    Eigen::Vector3d ee_center(last_robot_frames_[offset + 12], last_robot_frames_[offset + 13], last_robot_frames_[offset + 14]);
    Eigen::Vector3d gripper_left = ee_center + gripper_width * R.col(1) + finger_offset_ * R.col(2);
    Eigen::Vector3d gripper_right = ee_center - gripper_width * R.col(1) + finger_offset_ * R.col(2);

    transform.setOrigin(tf::Vector3(gripper_left[0], gripper_left[1], gripper_left[2]));
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "panda_link0", "panda_leftfinger"));

    transform.setOrigin(tf::Vector3(gripper_right[0], gripper_right[1], gripper_right[2]));
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "panda_link0", "panda_rightfinger"));
  }
}
