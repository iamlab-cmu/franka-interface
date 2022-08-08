#ifndef BASE_SHARED_MEMORY_HANDLER_HPP
#define BASE_SHARED_MEMORY_HANDLER_HPP

#include <franka_interface_msgs/action/execute_skill.hpp>
#include <franka_interface_msgs/msg/robot_state.hpp>
#include <franka_interface_msgs/msg/franka_interface_status.hpp>
#include <franka_interface_msgs/msg/run_loop_process_info_state.hpp>
#include <franka_interface_msgs/msg/sensor_data_group.hpp>

#include <rclcpp/rclcpp.hpp>
#include <array>
#include <vector>
#include <algorithm>

namespace franka_ros_interface
{
  class BaseSharedMemoryHandler
  {
    public:
      virtual ~BaseSharedMemoryHandler(){}

      virtual int loadSkillParametersIntoSharedMemory(const std::shared_ptr<const franka_interface_msgs::action::ExecuteSkill::Goal> goal) = 0;

      virtual void loadSensorDataGroupIntoSharedMemory(const franka_interface_msgs::msg::SensorDataGroup::SharedPtr sensor_data_group_ptr) = 0;

      virtual bool getSkillRunningFlagInSharedMemory() = 0;

      virtual int getDoneSkillIdInSharedMemory() = 0;

      virtual void setSkillCancelledFlagInSharedMemory(bool skill_cancelled_flag) = 0;

      virtual void setNewSkillDescriptionInSharedMemory(std::string description) = 0;

      virtual franka_interface_msgs::action::ExecuteSkill::Feedback getSkillFeedback() = 0;

      virtual franka_interface_msgs::action::ExecuteSkill::Result getSkillResult(int skill_id) = 0;

      virtual franka_interface_msgs::msg::RobotState getRobotState(std::array<double, 144> &robot_frames) = 0;

      virtual franka_interface_msgs::msg::FrankaInterfaceStatus getFrankaInterfaceStatus() = 0;

      virtual franka_interface_msgs::msg::RunLoopProcessInfoState getRunLoopProcessInfoState() = 0;
      
      virtual bool getNewSkillAvailableFlagInSharedMemory() = 0;

      virtual int getNewSkillIdInSharedMemory() = 0;

      virtual void incrementWatchdogCounter() = 0;

    protected:
      BaseSharedMemoryHandler(){}
  };
}  // namespace franka_ros_interface

#endif  // BASE_SHARED_MEMORY_HANDLER_HPP