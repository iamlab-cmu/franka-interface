#ifndef FRANKA_ROS_INTERFACE_EXECUTE_SKILL_ACTION_SERVER_H
#define FRANKA_ROS_INTERFACE_EXECUTE_SKILL_ACTION_SERVER_H

#include <iostream>
#include <thread>
#include <array>
#include <chrono>
#include <vector>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <franka_interface_msgs/action/execute_skill.hpp>
#include <franka_interface_msgs/msg/franka_interface_status.hpp>

#include "franka_ros_interface/visibility_control.h"
#include "franka_ros_interface/shared_memory_handler.h"

using namespace std::placeholders;

namespace franka_ros_interface  
{ 
  class ExecuteSkillActionServer : public rclcpp::Node
  {
    protected:

      // create messages that are used to published feedback/result
      franka_interface_msgs::action::ExecuteSkill::Feedback feedback_;
      franka_interface_msgs::action::ExecuteSkill::Result result_;

      franka_ros_interface::SharedMemoryHandler shared_memory_handler_;
      franka_interface_msgs::msg::FrankaInterfaceStatus franka_interface_status_;

      bool skill_cancelled_ = false;

    public:
      using ExecuteSkill = franka_interface_msgs::action::ExecuteSkill;
      using ExecuteSkillGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteSkill>;

      
      FRANKA_ROS_INTERFACE_PUBLIC
      explicit ExecuteSkillActionServer(const rclcpp::NodeOptions &options);

      ~ExecuteSkillActionServer(void){}

    private:
      rclcpp_action::Server<ExecuteSkill>::SharedPtr execute_skill_action_server_;

      rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteSkill::Goal> goal);
      rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<ExecuteSkillGoalHandle> goal_handle);
      void handle_accepted(const std::shared_ptr<ExecuteSkillGoalHandle> goal_handle);

      void execute(const std::shared_ptr<ExecuteSkillGoalHandle> goal_handle);
  };
}

#endif // FRANKA_ROS_INTERFACE_EXECUTE_SKILL_ACTION_SERVER_H
