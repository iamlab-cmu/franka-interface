#include "franka_ros_interface/execute_skill_action_server.h"

namespace franka_ros_interface
{
  ExecuteSkillActionServer::ExecuteSkillActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("execute_skill_action_server", options)
  {
    this->execute_skill_action_server_ = rclcpp_action::create_server<ExecuteSkill>(
      this,
      "~/execute_skill",
      std::bind(&ExecuteSkillActionServer::handle_goal, this, _1, _2),
      std::bind(&ExecuteSkillActionServer::handle_cancel, this, _1),
      std::bind(&ExecuteSkillActionServer::handle_accepted, this, _1));
  }

  rclcpp_action::GoalResponse ExecuteSkillActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteSkill::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "New Skill received: %s", goal->skill_description.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse ExecuteSkillActionServer::handle_cancel(
    const std::shared_ptr<ExecuteSkillGoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void ExecuteSkillActionServer::handle_accepted(const std::shared_ptr<ExecuteSkillGoalHandle> goal_handle)
  {
    std::thread{std::bind(&ExecuteSkillActionServer::execute, this, _1), goal_handle}.detach();
  }

  void ExecuteSkillActionServer::execute(const std::shared_ptr<ExecuteSkillGoalHandle> goal_handle)
  {

    const std::shared_ptr<const franka_interface_msgs::action::ExecuteSkill::Goal> goal = goal_handle->get_goal();
    std::shared_ptr<franka_interface_msgs::action::ExecuteSkill::Feedback> feedback = std::make_shared<ExecuteSkill::Feedback>();
    std::shared_ptr<franka_interface_msgs::action::ExecuteSkill::Result> result = std::make_shared<ExecuteSkill::Result>();

    franka_interface_status_ = shared_memory_handler_.getFrankaInterfaceStatus();
    if (!franka_interface_status_.is_ready) {
      RCLCPP_ERROR(this->get_logger(), "Franka Interface is not ready yet!");
      goal_handle->abort(result);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loading Skill Parameters into Shared Memory");
    rclcpp::Rate r(10.0);

    // Load skill parameters into shared memory and returns the skill_id
    int skill_id = shared_memory_handler_.loadSkillParametersIntoSharedMemory(goal);

    while(skill_id == -1) {
      r.sleep();

      skill_id = shared_memory_handler_.loadSkillParametersIntoSharedMemory(goal);
    }

    RCLCPP_INFO(this->get_logger(), "New Skill id = %d", skill_id);

    // Loop until skill is complete from shared memory or is cancelled
    int done_skill_id = shared_memory_handler_.getDoneSkillIdInSharedMemory();

    RCLCPP_INFO(this->get_logger(), "Done getting done_skill_id");

    // Ensure cancelled flag is set to false
    shared_memory_handler_.setSkillCancelledFlagInSharedMemory(false);
    while (done_skill_id < skill_id) {
      // this sleep is not necessary, the execution_feedback is computed at 10 Hz for demonstration purposes
      r.sleep();

      RCLCPP_DEBUG(this->get_logger(), "Getting Franka Interface Status");
      franka_interface_status_ = shared_memory_handler_.getFrankaInterfaceStatus();
      RCLCPP_DEBUG(this->get_logger(), "Done Getting Franka Interface Status");

      if (!franka_interface_status_.is_ready) {
        RCLCPP_INFO(this->get_logger(), "Franka Interface is not ready!");
        goal_handle->abort(result);
        break;
      }

      // check that client has not requested to cancel the skill
      if (goal_handle->is_canceling() || !rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "%s: Cancelled", goal->skill_description);
        // set the skill cancelled flag in shared memory to true and break from while loop
        skill_cancelled_ = true;
        goal_handle->canceled(result);
        shared_memory_handler_.setSkillCancelledFlagInSharedMemory(true);
        break;
      }

      // TODO fill in execution_feedback from shared memory
      RCLCPP_DEBUG(this->get_logger(), "Getting Skill Feedback");
      feedback_ = shared_memory_handler_.getSkillFeedback();

      if (feedback_.num_execution_feedback != 0){
        feedback = std::make_shared<ExecuteSkill::Feedback>(feedback_);

        // publish the feedback
        goal_handle->publish_feedback(feedback);
      }

      RCLCPP_DEBUG(this->get_logger(), "Getting Done Skill Id");
      done_skill_id = shared_memory_handler_.getDoneSkillIdInSharedMemory();
      RCLCPP_DEBUG(this->get_logger(), "Done Skill Id = %d", done_skill_id);
    }
    RCLCPP_INFO(this->get_logger(), "Skill Terminated Id = %d", skill_id);

    shared_memory_handler_.setNewSkillDescriptionInSharedMemory("");
    ExecuteSkillResultMessage result_msg = shared_memory_handler_.getSkillResultMessage(skill_id);
    result_ = shared_memory_handler_.getSkillResult(skill_id);
    result = std::make_shared<ExecuteSkill::Result>(result_);

    if(skill_cancelled_) {
      RCLCPP_INFO(this->get_logger(), "Skill was cancelled. Skill Id = %d", skill_id);
      skill_cancelled_ = false;
    } else if(result_msg.skill_result() == 0) {
      RCLCPP_INFO(this->get_logger(), "%s: Succeeded", goal->skill_description);
      // set the action state to succeeded
      goal_handle->succeed(result);
    } else if (result_msg.skill_result() == 1) {
      RCLCPP_INFO(this->get_logger(), "%s: Virtual Wall Collision", goal->skill_description);
      // set the action state to succeeded
      goal_handle->abort(result);
    } else if (result_msg.skill_result() == 2) {
      RCLCPP_INFO(this->get_logger(), "%s: Franka Exception ", goal->skill_description);
      // set the action state to succeeded
      goal_handle->abort(result);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Done Skill Id Error: Done Skill Id = %d, Skill Id = %d", done_skill_id, skill_id);
      goal_handle->abort(result);
    }
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(franka_ros_interface::ExecuteSkillActionServer)