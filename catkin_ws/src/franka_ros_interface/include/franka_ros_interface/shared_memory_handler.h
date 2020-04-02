#ifndef FRANKA_ROS_INTERFACE_SHARED_MEMORY_HANDLER_H
#define FRANKA_ROS_INTERFACE_SHARED_MEMORY_HANDLER_H

#include <franka_interface_msgs/ExecuteSkillAction.h> // Note: "Action" is appended
#include <franka_interface_msgs/RobotState.h>
#include <franka_interface_msgs/FrankaInterfaceStatus.h>
#include <franka_interface_msgs/RunLoopProcessInfoState.h>

#include "ros/ros.h" // For ROS::ERROR messages
#include <std_msgs/Float64.h>
#include "franka_interface_msgs/SensorData.h"
#include <franka_interface_msgs/SensorDataGroup.h>
#include <google/protobuf/message.h>
#include <robot_state_msg.pb.h>

#include <array>
#include <vector>
#include <algorithm>
#include <cassert>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

#include <franka-interface-common/definitions.h>
#include <franka-interface-common/franka_interface_state_info.h>
#include <franka-interface-common/run_loop_process_info.h>
#include <franka-interface-common/SharedMemoryInfo.h>

namespace franka_ros_interface
{
  class SharedMemoryHandler {
    public:
      SharedMemoryHandler();

      ~SharedMemoryHandler(void){}

      int loadSkillParametersIntoSharedMemory(const franka_interface_msgs::ExecuteSkillGoalConstPtr &goal);

      /**
       * The protocol for writing data to the shared memory is the following. Note that the shared memory is of type
       * unsigned int (uint_8) i.e. raw bytes.
       *
       * 1) First byte of the shared memory is set to 1, which indicates that there is new sensor data in the shared
       *    memory.
       *
       * 2) The second byte is the type of shared memory message type. This should be used to verify if the right
       *    message is being read by the franka-interface library running on control-PC. This is arbitrarily set for now.
       *    More importantly, this is just a single byte for now so the type value should lie between 0 and 255.
       *
       * 3) In the next 4 bytes (i.e. byte 2 to 6) we write the size of the sensor data message being written. We write
       *    the lowest byte (2) as the least significant byte of the integer size and so on.
       *
       * 4) From byte 6 onwards we write the raw proto data that we received via ROS from the workstation PC.
       *
       * @param ptr Pointer to the sensor data message to be written to the shared memory.
       * @param current_free_shared_memory_index
       */
      void loadSensorDataIntoSharedMemory(const franka_interface_msgs::SensorData &sensor_data, SensorBufferTypePtr sensor_buffer_ptr);
      
      /**
       * Will try to load all sensor data into shared memory. This method tries to acquire the lock to write to the
       * sensor data parts of shared memory. If successful, it writes the data into shared memory, else if it cannot
       * acquire the lock it does not do anything.
       */
      void tryToLoadSensorDataGroupIntoSharedMemory(const franka_interface_msgs::SensorDataGroup::ConstPtr &sensor_data_group_ptr);

      bool getSkillRunningFlagInSharedMemory();

      int getDoneSkillIdInSharedMemory();

      void setSkillPreemptedFlagInSharedMemory(bool skill_preempted_flag);

      void setNewSkillDescriptionInSharedMemory(std::string description);

      franka_interface_msgs::ExecuteSkillFeedback getSkillFeedback();

      franka_interface_msgs::ExecuteSkillResult getSkillResult(int skill_id);

      franka_interface_msgs::RobotState getRobotState(std::array<double, 144> &robot_frames);

      franka_interface_msgs::FrankaInterfaceStatus getFrankaInterfaceStatus();

      franka_interface_msgs::RunLoopProcessInfoState getRunLoopProcessInfoState();
      
      bool getNewSkillAvailableFlagInSharedMemory();

      int getNewSkillIdInSharedMemory();

      void incrementWatchdogCounter();

    private:

      SharedMemoryInfo shared_memory_info_;

      boost::interprocess::managed_shared_memory managed_shared_memory_;

      RunLoopProcessInfo *run_loop_process_info_;
      boost::interprocess::interprocess_mutex *run_loop_info_mutex_;

      FrankaInterfaceStateInfo *franka_interface_state_info_;
      boost::interprocess::interprocess_mutex *franka_interface_state_info_mutex_;

      boost::interprocess::interprocess_mutex *shared_memory_object_0_mutex_;
      boost::interprocess::interprocess_mutex *shared_memory_object_1_mutex_;

      boost::interprocess::interprocess_mutex *sensor_data_group_0_mutex_;

      boost::interprocess::interprocess_mutex *shared_execution_response_0_mutex_;
      boost::interprocess::interprocess_mutex *shared_execution_response_1_mutex_;

      boost::interprocess::interprocess_mutex *shared_current_robot_state_mutex_;

      boost::interprocess::shared_memory_object shared_memory_object_0_;
      boost::interprocess::shared_memory_object shared_memory_object_1_;
      boost::interprocess::shared_memory_object shared_sensor_data_0_;
      boost::interprocess::shared_memory_object shared_sensor_data_1_;
      boost::interprocess::shared_memory_object shared_execution_result_0_;
      boost::interprocess::shared_memory_object shared_execution_result_1_;
      boost::interprocess::shared_memory_object shared_current_robot_state_;

      boost::interprocess::mapped_region region_traj_params_0_;
      boost::interprocess::mapped_region region_feedback_controller_params_0_;
      boost::interprocess::mapped_region region_termination_params_0_;
      boost::interprocess::mapped_region region_timer_params_0_;

      boost::interprocess::mapped_region region_traj_params_1_;
      boost::interprocess::mapped_region region_feedback_controller_params_1_;
      boost::interprocess::mapped_region region_termination_params_1_;
      boost::interprocess::mapped_region region_timer_params_1_;

      boost::interprocess::mapped_region region_traj_sensor_data_0_;
      boost::interprocess::mapped_region region_feedback_controller_sensor_data_0_;
      boost::interprocess::mapped_region region_termination_sensor_data_0_;
      boost::interprocess::mapped_region region_timer_sensor_data_0_;
      boost::interprocess::mapped_region region_sensor_data_trajectory_generator_0_;
      boost::interprocess::mapped_region region_sensor_data_feedback_controller_0_;
      boost::interprocess::mapped_region region_sensor_data_termination_handler_0_;

      boost::interprocess::mapped_region region_traj_sensor_data_1_;
      boost::interprocess::mapped_region region_feedback_controller_sensor_data_1_;
      boost::interprocess::mapped_region region_termination_sensor_data_1_;
      boost::interprocess::mapped_region region_timer_sensor_data_1_;

      boost::interprocess::mapped_region execution_feedback_region_0_;
      boost::interprocess::mapped_region execution_result_region_0_;
      boost::interprocess::mapped_region execution_feedback_region_1_;
      boost::interprocess::mapped_region execution_result_region_1_;

      boost::interprocess::mapped_region shared_current_robot_region_;

      SharedBufferTypePtr trajectory_generator_buffer_0_;
      SharedBufferTypePtr feedback_controller_buffer_0_;
      SharedBufferTypePtr termination_handler_buffer_0_;
      SharedBufferTypePtr timer_buffer_0_;
      SharedBufferTypePtr trajectory_generator_buffer_1_;
      SharedBufferTypePtr feedback_controller_buffer_1_;
      SharedBufferTypePtr termination_handler_buffer_1_;
      SharedBufferTypePtr timer_buffer_1_;

      SensorBufferTypePtr sensor_data_trajectory_generator_buffer_0_ ;
      SensorBufferTypePtr sensor_data_feedback_controller_buffer_0_ ;
      SensorBufferTypePtr sensor_data_termination_handler_buffer_0_ ;

      SharedBufferTypePtr execution_feedback_buffer_0_;
      SharedBufferTypePtr execution_result_buffer_0_;
      SharedBufferTypePtr execution_feedback_buffer_1_;
      SharedBufferTypePtr execution_result_buffer_1_;

      SharedBufferTypePtr current_robot_state_buffer_;

      int getCurrentFreeSharedMemoryIndexInSharedMemoryUnprotected();
      int getCurrentSkillIdInSharedMemoryUnprotected();
      int getDoneSkillIdInSharedMemoryUnprotected();
      bool getNewSkillAvailableFlagInSharedMemoryUnprotected();
      void setNewSkillFlagInSharedMemoryUnprotected(bool new_skill_flag);
      int getNewSkillIdInSharedMemoryUnprotected();
      void setNewSkillIdInSharedMemoryUnprotected(int new_skill_id);
      void setNewSkillDescriptionInSharedMemoryUnprotected(std::string description);
      void setNewSkillTypeInSharedMemoryUnprotected(int new_skill_type);
      void setNewMetaSkillIdInSharedMemoryUnprotected(int new_meta_skill_id);
      void setNewMetaSkillTypeInSharedMemoryUnprotected(int new_meta_skill_type);
      void setResultSkillIdInSharedMemoryUnprotected(int result_skill_id);

      void loadSensorDataUnprotected(const franka_interface_msgs::ExecuteSkillGoalConstPtr &goal, int current_free_shared_memory_index);
      void loadTrajGenParamsUnprotected(const franka_interface_msgs::ExecuteSkillGoalConstPtr &goal, int current_free_shared_memory_index);
      void loadFeedbackControllerParamsUnprotected(const franka_interface_msgs::ExecuteSkillGoalConstPtr &goal, int current_free_shared_memory_index);
      void loadTerminationParamsUnprotected(const franka_interface_msgs::ExecuteSkillGoalConstPtr &goal, int current_free_shared_memory_index);
      void loadTimerParamsUnprotected(const franka_interface_msgs::ExecuteSkillGoalConstPtr &goal, int current_free_shared_memory_index);
  };
}


#endif // FRANKA_ROS_INTERFACE_SHARED_MEMORY_HANDLER_H
