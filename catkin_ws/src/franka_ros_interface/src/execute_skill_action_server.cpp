#include "franka_ros_interface/execute_skill_action_server.h"

namespace franka_ros_interface
{
  ExecuteSkillActionServer::ExecuteSkillActionServer(std::string name) :  nh_("~"),
                                                                          as_(nh_, name, boost::bind(&ExecuteSkillActionServer::executeCB, this, _1), false),
                                                                          action_name_(name),
                                                                          shared_memory_handler_() {
    nh_.param("publish_frequency", publish_frequency_, (double) 10.0);

    as_.start();

    ROS_INFO("Action Lib Started");

  }

  void ExecuteSkillActionServer::executeCB(const franka_interface_msgs::ExecuteSkillGoalConstPtr &goal) {
    ros::Rate r(publish_frequency_);

    ROS_INFO("=======");

    ROS_INFO("New Skill received: %s", goal->skill_description.c_str());

    franka_interface_status_ = shared_memory_handler_.getFrankaInterfaceStatus();
    if (!franka_interface_status_.is_ready) {
      ROS_ERROR("FrankaInterface is not ready yet!");
      as_.setAborted();
      return;
    }

    // Load skill parameters into shared memory and returns the skill_id
    int skill_id = shared_memory_handler_.loadSkillParametersIntoSharedMemory(goal);

    while(skill_id == -1) {
      r.sleep();

      skill_id = shared_memory_handler_.loadSkillParametersIntoSharedMemory(goal);
    }

    ROS_INFO("New Skill id = %d", skill_id);

    // Loop until skill is complete from shared memory or is preempted
    int done_skill_id = shared_memory_handler_.getDoneSkillIdInSharedMemory();

    ROS_INFO("Done getting done_skill_id");

    // Ensure preempted flag is set to false
    shared_memory_handler_.setSkillPreemptedFlagInSharedMemory(false);
    while (done_skill_id < skill_id) {
      // this sleep is not necessary, the execution_feedback is computed at 10 Hz for demonstration purposes
      r.sleep();

      ROS_DEBUG("getting franka_interface status");
      franka_interface_status_ = shared_memory_handler_.getFrankaInterfaceStatus();
      ROS_DEBUG("getting franka_interface status done");

      if (!franka_interface_status_.is_ready) {
        ROS_INFO("franka_interface status is not ready\n");
        as_.setAborted();
        break;
      }

      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        shared_memory_handler_.setSkillPreemptedFlagInSharedMemory(true);
        break;
      }

      // TODO(jacky): uncomment these once we separate their values from run_loop_info
      // ROS_DEBUG("getting skill available");
      // ROS_DEBUG("get_new_skill_available = %d", shared_memory_handler_.getNewSkillAvailableFlagInSharedMemory());
      // ROS_DEBUG("getting done skill available");
      // ROS_DEBUG("get_done_skill_id = %d", shared_memory_handler_.getDoneSkillIdInSharedMemory());
      // ROS_DEBUG("getting new skill available");
      // ROS_DEBUG("get_new_skill_id = %d", shared_memory_handler_.getNewSkillIdInSharedMemory());

      // TODO fill in execution_feedback from shared memory
      ROS_DEBUG("getting skill feedback");
      feedback_ = shared_memory_handler_.getSkillFeedback();
      // TODO(jacky) remove this check (and the logic in shared mem handler) once run_loop_info is no longer used
      if (feedback_.num_execution_feedback == -1) continue;

      // publish the feedback
      as_.publishFeedback(feedback_);
      
      ROS_DEBUG("getting done skill id");
      done_skill_id = shared_memory_handler_.getDoneSkillIdInSharedMemory();
      ROS_DEBUG("done skill id = %d", done_skill_id);
    }
    ROS_INFO("Skill terminated id = %d", skill_id);

    shared_memory_handler_.setNewSkillDescriptionInSharedMemory("");
    result_ = shared_memory_handler_.getSkillResult(skill_id);
    if (done_skill_id != -1 && (done_skill_id == skill_id || done_skill_id == skill_id + 1)) {
      // Get execution result from shared memory
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    } else if (as_.isPreemptRequested()) {
      ROS_INFO("Skill was preempted. skill_id = %d", skill_id);
    } else {
      ROS_WARN("done_skill_id error: done_skill_id = %d, skill_id = %d", done_skill_id, skill_id);
    }
  }
}
