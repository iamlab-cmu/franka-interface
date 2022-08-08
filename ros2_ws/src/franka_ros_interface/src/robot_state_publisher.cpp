#include "franka_ros_interface/robot_state_publisher.h"

namespace franka_ros_interface
{
  RobotStatePublisher::RobotStatePublisher() :  Node("robot_state_publisher")
  {
    robot_state_pub_ = this->create_publisher<franka_interface_msgs::msg::RobotState>("~/robot_state", 100);
    timer_ = this->create_wall_timer(10ms, std::bind(&RobotStatePublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Robot State Publisher Started");
  }

  void RobotStatePublisher::timer_callback()
  {
    franka_interface_msgs::msg::RobotState robot_state_ = shared_memory_handler_.getRobotState(last_robot_frames_);
    robot_state_.header.stamp = this->get_clock()->now();

    if (robot_state_.is_fresh) {
      robot_state_pub_->publish(robot_state_);

      has_seen_one_robot_state_ = true;
      last_robot_state_ = robot_state_;
      last_robot_state_.is_fresh = false;

      BroadcastRobotFrames();
    } else {
      if (has_seen_one_robot_state_) {
        robot_state_pub_->publish(last_robot_state_);
      }
    }
  }

  void RobotStatePublisher::BroadcastRobotFrames()
  {
    int offset;
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "panda_link0";
    Eigen::Matrix3d R;

    for (long unsigned int frame = 0; frame < frame_names_.size(); frame++)
    {
      offset = frame * 16;
      t.transform.translation.x = last_robot_frames_[offset + 12];
      t.transform.translation.y = last_robot_frames_[offset + 13];
      t.transform.translation.z = last_robot_frames_[offset + 14];
      
      R << last_robot_frames_[offset + 0], last_robot_frames_[offset + 4], last_robot_frames_[offset + 8],
            last_robot_frames_[offset + 1], last_robot_frames_[offset + 5], last_robot_frames_[offset + 9], 
            last_robot_frames_[offset + 2], last_robot_frames_[offset + 6], last_robot_frames_[offset + 10];
      Eigen::Quaterniond eigen_q(R);

      tf2::Quaternion q(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w());
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      t.header.stamp = this->get_clock()->now();
      t.child_frame_id = frame_names_[frame];

      tf_broadcaster_->sendTransform(t);
    }
    
    // panda_hand position from kFlange
    offset = 7 * 16;

    t.transform.translation.x = last_robot_frames_[offset + 12];
    t.transform.translation.y = last_robot_frames_[offset + 13];
    t.transform.translation.z = last_robot_frames_[offset + 14];

    // panda_hand orientation from kEndEffector
    offset = 8 * 16;
    R << last_robot_frames_[offset + 0], last_robot_frames_[offset + 4], last_robot_frames_[offset + 8],
          last_robot_frames_[offset + 1], last_robot_frames_[offset + 5], last_robot_frames_[offset + 9], 
          last_robot_frames_[offset + 2], last_robot_frames_[offset + 6], last_robot_frames_[offset + 10];
    Eigen::Quaterniond eigen_q(R);

    tf2::Quaternion q(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w());
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    t.header.stamp = this->get_clock()->now();
    t.child_frame_id = "panda_hand";

    tf_broadcaster_->sendTransform(t);

    // For grippers
    offset = 7 * 16;
    double gripper_width = last_robot_state_.gripper_width/2.;

    Eigen::Vector3d ee_center(last_robot_frames_[offset + 12], last_robot_frames_[offset + 13], last_robot_frames_[offset + 14]);
    Eigen::Vector3d gripper_left = ee_center + gripper_width * R.col(1) + finger_offset_ * R.col(2);
    Eigen::Vector3d gripper_right = ee_center - gripper_width * R.col(1) + finger_offset_ * R.col(2);

    t.transform.translation.x = gripper_left[0];
    t.transform.translation.y = gripper_left[1];
    t.transform.translation.z = gripper_left[2];
    t.header.stamp = this->get_clock()->now();
    t.child_frame_id = "panda_leftfinger";
    tf_broadcaster_->sendTransform(t);

    t.transform.translation.x = gripper_right[0];
    t.transform.translation.y = gripper_right[1];
    t.transform.translation.z = gripper_right[2];
    t.header.stamp = this->get_clock()->now();
    t.child_frame_id = "panda_rightfinger";
    tf_broadcaster_->sendTransform(t);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<franka_ros_interface::RobotStatePublisher>());
  rclcpp::shutdown();

  return 0;
}