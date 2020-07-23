#include "franka_ros_interface/shared_memory_handler.h"

namespace franka_ros_interface
{

  SharedMemoryHandler::SharedMemoryHandler() : shared_memory_info_()
  {
    managed_shared_memory_ = boost::interprocess::managed_shared_memory(
        boost::interprocess::open_only, shared_memory_info_.getSharedMemoryNameForObjects().c_str());

    // Get RunLoopProcessInfo from the the shared memory segment.
    std::pair<RunLoopProcessInfo*, std::size_t> run_loop_process_info_pair = \
        managed_shared_memory_.find<RunLoopProcessInfo> (shared_memory_info_.getRunLoopInfoObjectName().c_str());
    run_loop_process_info_ = run_loop_process_info_pair.first;

    // Make sure the process info object can be found in memory.
    assert(run_loop_process_info_ != 0);

    // Get mutex for ProcessInfo from the shared memory segment.
    std::pair<boost::interprocess::interprocess_mutex *, std::size_t> run_loop_info_mutex_pair = \
                                managed_shared_memory_.find<boost::interprocess::interprocess_mutex>
                                (shared_memory_info_.getRunLoopInfoMutexName().c_str());
    run_loop_info_mutex_ = run_loop_info_mutex_pair.first;
    assert(run_loop_info_mutex_ != 0);

    // Get FrankaInterfaceStateInfo from the the shared memory segment.
    std::pair<FrankaInterfaceStateInfo*, std::size_t> franka_interface_state_info_pair = \
        managed_shared_memory_.find<FrankaInterfaceStateInfo> (shared_memory_info_.getFrankaInterfaceStateInfoObjectName().c_str());
    franka_interface_state_info_ = franka_interface_state_info_pair.first;

    // Make sure the process info object can be found in memory.
    assert(franka_interface_state_info_ != 0);

    // Get mutex for ProcessInfo from the shared memory segment.
    std::pair<boost::interprocess::interprocess_mutex *, std::size_t> franka_interface_state_info_mutex_pair = \
                                managed_shared_memory_.find<boost::interprocess::interprocess_mutex>
                                (shared_memory_info_.getFrankaInterfaceStateInfoMutexName().c_str());
    franka_interface_state_info_mutex_ = franka_interface_state_info_mutex_pair.first;
    assert(franka_interface_state_info_mutex_ != 0);

    // Get mutex for buffer 0 from the shared memory segment.
    std::pair<boost::interprocess::interprocess_mutex *, std::size_t> shared_memory_object_0_mutex_pair = \
                                managed_shared_memory_.find<boost::interprocess::interprocess_mutex>
                                (shared_memory_info_.getParameterMemoryMutexName(0).c_str());
    shared_memory_object_0_mutex_ = shared_memory_object_0_mutex_pair.first;
    assert(shared_memory_object_0_mutex_ != 0);

    /**
     * Open shared memory region for parameter buffer 0.
     */
    shared_memory_object_0_ = boost::interprocess::shared_memory_object(
        boost::interprocess::open_only,
        shared_memory_info_.getSharedMemoryNameForParameters(0).c_str(),
        boost::interprocess::read_write
    );

    // Allocate regions for each parameter array
    region_traj_params_0_ = boost::interprocess::mapped_region(
        shared_memory_object_0_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForTrajectoryParameters(),
        shared_memory_info_.getSizeForTrajectoryParameters()
        );
    trajectory_generator_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(region_traj_params_0_.get_address());
    region_feedback_controller_params_0_ = boost::interprocess::mapped_region(
        shared_memory_object_0_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForFeedbackControllerParameters(),
        shared_memory_info_.getSizeForFeedbackControllerParameters()
        );
    feedback_controller_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(region_feedback_controller_params_0_.get_address());
    region_termination_params_0_ = boost::interprocess::mapped_region(
        shared_memory_object_0_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForTerminationParameters(),
        shared_memory_info_.getSizeForTerminationParameters()
    );

    termination_handler_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(region_termination_params_0_.get_address());
    region_timer_params_0_ = boost::interprocess::mapped_region(
        shared_memory_object_0_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForTimerParameters(),
        shared_memory_info_.getSizeForTimerParameters()
    );
    timer_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(region_timer_params_0_.get_address());
    region_sensor_data_trajectory_generator_0_ =  boost::interprocess::mapped_region(
        shared_memory_object_0_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForSensorDataTrajectoryGenerator(),
        shared_memory_info_.getSizeForSensorData()
    );
    sensor_data_trajectory_generator_buffer_0_ = reinterpret_cast<SensorBufferTypePtr>(region_sensor_data_trajectory_generator_0_.get_address());
    region_sensor_data_feedback_controller_0_ =  boost::interprocess::mapped_region(
        shared_memory_object_0_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForSensorDataFeedbackController(),
        shared_memory_info_.getSizeForSensorData()
    );
    sensor_data_feedback_controller_buffer_0_ = reinterpret_cast<SensorBufferTypePtr>(region_sensor_data_feedback_controller_0_.get_address());
    region_sensor_data_termination_handler_0_ =  boost::interprocess::mapped_region(
        shared_memory_object_0_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForSensorDataTerminationHandler(),
        shared_memory_info_.getSizeForSensorData()
    );
    sensor_data_termination_handler_buffer_0_ = reinterpret_cast<SensorBufferTypePtr>(region_sensor_data_termination_handler_0_.get_address());

    // Get mutex for buffer 1 from the shared memory segment.
    std::pair<boost::interprocess::interprocess_mutex *, std::size_t> shared_memory_object_1_mutex_pair = \
                                managed_shared_memory_.find<boost::interprocess::interprocess_mutex>
                                (shared_memory_info_.getParameterMemoryMutexName(1).c_str());
    shared_memory_object_1_mutex_ = shared_memory_object_1_mutex_pair.first;
    assert(shared_memory_object_1_mutex_ != 0);

    /**
     * Open shared memory region for parameter buffer 1.
     */
    shared_memory_object_1_ = boost::interprocess::shared_memory_object(
        boost::interprocess::open_only,
        shared_memory_info_.getSharedMemoryNameForParameters(1).c_str(),
        boost::interprocess::read_write
        );

    // Allocate regions for each parameter array
    region_traj_params_1_ =  boost::interprocess::mapped_region(
        shared_memory_object_1_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForTrajectoryParameters(),
        sizeof(SharedBufferType) * shared_memory_info_.getSizeForTrajectoryParameters()
        );
    trajectory_generator_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(region_traj_params_1_.get_address());
    region_feedback_controller_params_1_ = boost::interprocess::mapped_region(
        shared_memory_object_1_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForFeedbackControllerParameters(),
        shared_memory_info_.getSizeForFeedbackControllerParameters()
        );
    feedback_controller_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(region_feedback_controller_params_1_.get_address());
    region_termination_params_1_ = boost::interprocess::mapped_region(
        shared_memory_object_1_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForTerminationParameters(),
        shared_memory_info_.getSizeForTerminationParameters()
        );
    termination_handler_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(region_termination_params_1_.get_address());
    region_timer_params_1_ = boost::interprocess::mapped_region(
        shared_memory_object_1_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForTimerParameters(),
        shared_memory_info_.getSizeForTimerParameters()
        );
    timer_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(region_timer_params_1_.get_address());

    std::pair<boost::interprocess::interprocess_mutex *, std::size_t> sensor_data_0_mutex_pair = \
      managed_shared_memory_.find<boost::interprocess::interprocess_mutex>
              (shared_memory_info_.getSensorDataGroupMutexName(0).c_str());
    sensor_data_group_0_mutex_ = sensor_data_0_mutex_pair.first;
    assert(sensor_data_group_0_mutex_ != 0);

    /**
     * Open shared memory region for sensor data buffer 1.
     */

    // Get mutex for execution response buffer 0 from the shared memory segment.
    std::pair<boost::interprocess::interprocess_mutex *, std::size_t> shared_execution_response_0_mutex_pair = \
                                managed_shared_memory_.find<boost::interprocess::interprocess_mutex>
                                (shared_memory_info_.getExecutionResponseMutexName(0).c_str());
    shared_execution_response_0_mutex_ = shared_execution_response_0_mutex_pair.first;
    assert(shared_execution_response_0_mutex_ != 0);

    /**
     * Open shared memory region for execution response buffer 0.
     */
    shared_execution_result_0_ = boost::interprocess::shared_memory_object(
        boost::interprocess::open_only,
        shared_memory_info_.getSharedMemoryNameForResults(0).c_str(),
        boost::interprocess::read_write
    );

    execution_feedback_region_0_ =  boost::interprocess::mapped_region(
        shared_execution_result_0_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForExecutionFeedbackData(),
        shared_memory_info_.getSizeForExecutionFeedbackData()
    );
    execution_feedback_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(execution_feedback_region_0_.get_address());
    execution_result_region_0_ = boost::interprocess::mapped_region(
        shared_execution_result_0_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForExecutionResultData(),
        shared_memory_info_.getSizeForExecutionResultData()
        );
    execution_result_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(execution_result_region_0_.get_address());

    // Get mutex for execution response buffer 1 from the shared memory segment.
    std::pair<boost::interprocess::interprocess_mutex *, std::size_t> shared_execution_response_1_mutex_pair = \
                                managed_shared_memory_.find<boost::interprocess::interprocess_mutex>
                                (shared_memory_info_.getExecutionResponseMutexName(1).c_str());
    shared_execution_response_1_mutex_ = shared_execution_response_1_mutex_pair.first;
    assert(shared_execution_response_1_mutex_ != 0);

    /**
     * Open shared memory region for execution response buffer 1.
     */
    shared_execution_result_1_ = boost::interprocess::shared_memory_object(
        boost::interprocess::open_only,
        shared_memory_info_.getSharedMemoryNameForResults(1).c_str(),
        boost::interprocess::read_write
    );

    execution_feedback_region_1_ =  boost::interprocess::mapped_region(
        shared_execution_result_1_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForExecutionFeedbackData(),
        shared_memory_info_.getSizeForExecutionFeedbackData()
    );
    execution_feedback_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(execution_feedback_region_1_.get_address());
    execution_result_region_1_ = boost::interprocess::mapped_region(
        shared_execution_result_1_,
        boost::interprocess::read_write,
        shared_memory_info_.getOffsetForExecutionResultData(),
        shared_memory_info_.getSizeForExecutionResultData()
    );
    execution_result_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(execution_result_region_1_.get_address());

    // Get mutex for current robot state buffer
    std::pair<boost::interprocess::interprocess_mutex *, std::size_t> shared_current_robot_state_mutex_pair = \
                                managed_shared_memory_.find<boost::interprocess::interprocess_mutex>
                                (shared_memory_info_.getCurrentRobotStateMutexName().c_str());
    shared_current_robot_state_mutex_ = shared_current_robot_state_mutex_pair.first;
    assert(shared_current_robot_state_mutex_ != 0);

    /**
     * Open shared memory region for current robot state buffer
     */
    shared_current_robot_state_ = boost::interprocess::shared_memory_object(
        boost::interprocess::open_only,
        shared_memory_info_.getSharedMemoryNameForCurrentRobotState().c_str(),
        boost::interprocess::read_only
    );

    shared_current_robot_region_ =  boost::interprocess::mapped_region(
        shared_current_robot_state_,
        boost::interprocess::read_only,
        shared_memory_info_.getOffsetForExecutionFeedbackData(),
        shared_memory_info_.getSizeForExecutionFeedbackData()
    );
    current_robot_state_buffer_ = reinterpret_cast<SharedBufferTypePtr>(shared_current_robot_region_.get_address());

  }

  int SharedMemoryHandler::loadSkillParametersIntoSharedMemory(const franka_interface_msgs::ExecuteSkillGoalConstPtr &goal)
  {
    // Grab lock of run_loop_info_mutex_ to see if we can load the new skill parameters into the shared memory
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_);

    bool new_skill_flag = getNewSkillAvailableFlagInSharedMemoryUnprotected();

    // Return -1 if the new_skill_flag is already set to true, which means that another new skill has already been loaded
    // into the shared memory.
    if(new_skill_flag)
    {
      return -1;
    }

    int current_skill_id = getCurrentSkillIdInSharedMemoryUnprotected();
    int new_skill_id = getNewSkillIdInSharedMemoryUnprotected();

    ROS_DEBUG("Current skill id: %d", current_skill_id);
    ROS_DEBUG("New skill id: %d", new_skill_id);

    if(current_skill_id != new_skill_id)
    {
      ROS_ERROR("Error with the current_skill_id and new_skill_id. current_skill_id = %d, new_skill_id = %d", current_skill_id, new_skill_id);
      return -1;
    }

    new_skill_id += 1;

    int current_free_shared_memory_index = getCurrentFreeSharedMemoryIndexInSharedMemoryUnprotected();

    // Grab lock of the free shared memory
    if(current_free_shared_memory_index == 0)
    {
      // Grab lock of shared_memory_object_0_mutex_ to make sure no one else can modify shared_memory_0_
      boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> shared_memory_object_0_lock(*shared_memory_object_0_mutex_);

      // Load all of the data into shared_memory_0_
      loadSensorDataUnprotected(goal, 0);

      loadTrajGenParamsUnprotected(goal, 0);
      loadFeedbackControllerParamsUnprotected(goal, 0);
      loadTerminationParamsUnprotected(goal, 0);
      loadTimerParamsUnprotected(goal, 0);

      // The lock of the shared_memory_object_0_mutex_ should be released automatically
    }
    else if (current_free_shared_memory_index == 1)
    {
      // Grab lock of shared_memory_object_1_mutex_ to make sure no one else can modify shared_memory_1_
      boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> shared_memory_object_1_lock(*shared_memory_object_1_mutex_);

      // Load all of the data into shared_memory_1_
      loadSensorDataUnprotected(goal, 1);
      loadTrajGenParamsUnprotected(goal, 1);
      loadFeedbackControllerParamsUnprotected(goal, 1);
      loadTerminationParamsUnprotected(goal, 1);
      loadTimerParamsUnprotected(goal, 1);

      // The lock of the shared_memory_object_1_mutex_ should be released automatically
    }

    setNewSkillIdInSharedMemoryUnprotected(new_skill_id);
    setNewSkillTypeInSharedMemoryUnprotected(goal->skill_type);
    setNewSkillDescriptionInSharedMemoryUnprotected(goal->skill_description);
    setNewMetaSkillTypeInSharedMemoryUnprotected(goal->meta_skill_type);
    setNewMetaSkillIdInSharedMemoryUnprotected(goal->meta_skill_id);

    // Set the new skill flag in shared memory to true to signal that a new skill has been loaded into the current free shared memory.
    setNewSkillFlagInSharedMemoryUnprotected(true);

    // Return the skill_id of the current skill
    return new_skill_id;

    // The lock of the run_loop_info_mutex_ should be released automatically
  }

  bool SharedMemoryHandler::getSkillRunningFlagInSharedMemory()
  {
    bool skill_running_flag;
    {
      // Grab the lock of the run_loop_info_mutex_
      boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_);

      skill_running_flag = run_loop_process_info_->get_is_running_skill();

      // The lock of the run_loop_info_mutex_ should be released automatically
    }

    // Return the skill_running_flag
    return skill_running_flag;
  }

  int SharedMemoryHandler::getDoneSkillIdInSharedMemory()
  {
    // Grab the lock of the run_loop_info_mutex_
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_, boost::interprocess::defer_lock);
    if (run_loop_info_lock.try_lock()) {
      // Return the done_skill_id
      return getDoneSkillIdInSharedMemoryUnprotected();
    } else {
      return -1;
    }

    // The lock of the run_loop_info_mutex_ should be released automatically
  }

  void SharedMemoryHandler::setSkillPreemptedFlagInSharedMemory(bool skill_preempted_flag)
  {
    // Grab the lock of the run_loop_info_mutex_
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_);

    // Set skill_preempted_ in run_loop_process_info_ to the input skill_preempted_flag
    run_loop_process_info_->set_skill_preempted(skill_preempted_flag);

    // The lock of the run_loop_info_mutex_ should be released automatically
  }

  void SharedMemoryHandler::setNewSkillDescriptionInSharedMemory(std::string description)
  {
    // Grab the lock of the run_loop_info_mutex_
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_);

    setNewSkillDescriptionInSharedMemoryUnprotected(description);

    // The lock of the run_loop_info_mutex_ should be released automatically
  }

  franka_interface_msgs::ExecuteSkillFeedback SharedMemoryHandler::getSkillFeedback()
  {
    franka_interface_msgs::ExecuteSkillFeedback feedback;

    // Grab the lock of the run_loop_info_mutex_
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_, boost::interprocess::defer_lock);

    if (run_loop_info_lock.try_lock()) {

      int current_free_shared_feedback_index = run_loop_process_info_->get_current_free_shared_feedback_index();

      if(current_free_shared_feedback_index == 0)
      {
        // Grab lock of shared_execution_response_0_mutex_ to make sure no one else can modify 0
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> shared_execution_response_0_lock(*shared_execution_response_0_mutex_);

        int num_execution_feedback = static_cast<int>(execution_feedback_buffer_0_[0]);

        std::vector<SharedBufferType> execution_feedback(execution_feedback_buffer_0_ + 1, execution_feedback_buffer_0_ + num_execution_feedback + 1);

        feedback.num_execution_feedback = num_execution_feedback;
        feedback.execution_feedback = execution_feedback;

        // The lock of the shared_execution_response_0_mutex_ should be released automatically
      }
      else if(current_free_shared_feedback_index == 1)
      {
        // Grab lock of shared_execution_response_1_mutex_ to make sure no one else can modify 1
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> shared_execution_response_1_lock(*shared_execution_response_1_mutex_);

        int num_execution_feedback = static_cast<int>(execution_feedback_buffer_1_[0]);

        std::vector<SharedBufferType> execution_feedback(execution_feedback_buffer_1_ + 1, execution_feedback_buffer_1_ + num_execution_feedback + 1);

        feedback.num_execution_feedback = num_execution_feedback;
        feedback.execution_feedback = execution_feedback;

        // The lock of the shared_execution_response_1_mutex_ should be released automatically
      }
    } else {
      feedback.num_execution_feedback = -1;
    }

    return feedback;

    // The lock of the run_loop_info_mutex_ should be released automatically
  }

  franka_interface_msgs::ExecuteSkillResult SharedMemoryHandler::getSkillResult(int skill_id)
  {
    franka_interface_msgs::ExecuteSkillResult result;

    // Grab the lock of the run_loop_info_mutex_
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_);

    int result_memory_index = skill_id % 2;

    if(result_memory_index == 0)
    {
      int num_execution_result = static_cast<int>(execution_result_buffer_0_[0]);

      std::vector<SharedBufferType> execution_result(execution_result_buffer_0_ + 1, execution_result_buffer_0_ + num_execution_result + 1);

      result.num_execution_result = num_execution_result;
      result.execution_result = execution_result;

      // The lock of the shared_execution_response_0_mutex_ should be released automatically
    }
    else if(result_memory_index == 1)
    {
      int num_execution_result = static_cast<int>(execution_result_buffer_1_[0]);

      std::vector<SharedBufferType> execution_result(execution_result_buffer_1_ + 1, execution_result_buffer_1_ + num_execution_result + 1);

      result.num_execution_result = num_execution_result;
      result.execution_result = execution_result;

      // The lock of the shared_execution_response_1_mutex_ should be released automatically
    }

    setResultSkillIdInSharedMemoryUnprotected(skill_id);

    return result;

    // The lock of the run_loop_info_mutex_ should be released automatically
  }

  franka_interface_msgs::RobotState SharedMemoryHandler::getRobotState(std::array<double, 144> &robot_frames) {
    franka_interface_msgs::RobotState robot_state;

    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> current_robot_state_lock(*shared_current_robot_state_mutex_, boost::interprocess::defer_lock);
    if (current_robot_state_lock.try_lock()) {
      robot_state.header.stamp = ros::Time::now();

      int num_bytes = (current_robot_state_buffer_[0] + 
                      (current_robot_state_buffer_[1] << 8) + 
                      (current_robot_state_buffer_[2] << 16) + 
                      (current_robot_state_buffer_[3] << 24));

      RobotStateMessage robot_state_msg;
      if(robot_state_msg.ParseFromArray(current_robot_state_buffer_ + 4, num_bytes) == false) {
        std::cout << "ParsingFromArray Exception occurred.\n";
        robot_state.is_fresh = false;
        return robot_state;
      }

      for (int i = 0; i < 16; i++) {
        robot_state.pose_desired[i] = robot_state_msg.pose_desired(i);
        robot_state.O_T_EE[i] = robot_state_msg.o_t_ee(i);
        robot_state.O_T_EE_d[i] = robot_state_msg.o_t_ee_d(i);
        robot_state.O_T_EE_c[i] = robot_state_msg.o_t_ee_c(i);
        robot_state.F_T_EE[i] = robot_state_msg.f_t_ee(i);
        robot_state.EE_T_K[i] = robot_state_msg.ee_t_k(i);
      }

      robot_state.m_ee = robot_state_msg.m_ee();
      robot_state.m_load = robot_state_msg.m_load();
      robot_state.m_total = robot_state_msg.m_total();

      for (int i = 0; i < 9; i++) {
        robot_state.I_ee[i] = robot_state_msg.i_ee(i);
        robot_state.I_load[i] = robot_state_msg.i_load(i);
        robot_state.I_total[i] = robot_state_msg.i_total(i);
      }

      for (int i = 0; i < 3; i++) {
        robot_state.F_x_Cee[i] = robot_state_msg.f_x_cee(i);
        robot_state.F_x_Cload[i] = robot_state_msg.f_x_cload(i);
        robot_state.F_x_Ctotal[i] = robot_state_msg.f_x_ctotal(i);
      }

      for (int i = 0; i < 2; i++) {
        robot_state.elbow[i] = robot_state_msg.elbow(i);
        robot_state.elbow_d[i] = robot_state_msg.elbow_d(i);
        robot_state.elbow_c[i] = robot_state_msg.elbow_c(i);
        robot_state.delbow_c[i] = robot_state_msg.delbow_c(i);
        robot_state.ddelbow_c[i] = robot_state_msg.ddelbow_c(i);
      }    

      for (int i = 0; i < 7; i++) {
        robot_state.tau_J[i] = robot_state_msg.tau_j(i);
        robot_state.tau_J_d[i] = robot_state_msg.tau_j_d(i);
        robot_state.dtau_J[i] = robot_state_msg.dtau_j(i);
        robot_state.q[i] = robot_state_msg.q(i);
        robot_state.q_d[i] = robot_state_msg.q_d(i);
        robot_state.dq[i] = robot_state_msg.dq(i);
        robot_state.dq_d[i] = robot_state_msg.dq_d(i);
        robot_state.ddq_d[i] = robot_state_msg.ddq_d(i);
        robot_state.joint_contact[i] = robot_state_msg.joint_contact(i);
        robot_state.joint_collision[i] = robot_state_msg.joint_collision(i);
        robot_state.tau_ext_hat_filtered[i] = robot_state_msg.tau_ext_hat_filtered(i);
        robot_state.theta[i] = robot_state_msg.theta(i);
        robot_state.dtheta[i] = robot_state_msg.dtheta(i);
      }

      for (int i = 0; i < 6; i++) {
        robot_state.cartesian_contact[i] = robot_state_msg.cartesian_contact(i);
        robot_state.cartesian_collision[i] = robot_state_msg.cartesian_collision(i);
        robot_state.O_F_ext_hat_K[i] = robot_state_msg.o_f_ext_hat_k(i);
        robot_state.K_F_ext_hat_K[i] = robot_state_msg.k_f_ext_hat_k(i);
        robot_state.O_dP_EE_d[i] = robot_state_msg.o_dp_ee_d(i);
        robot_state.O_dP_EE_c[i] = robot_state_msg.o_dp_ee_c(i);
        robot_state.O_ddP_EE_c[i] = robot_state_msg.o_ddp_ee_c(i);
      }

      for (int i = 0; i < 144; i++) {
        robot_frames[i] = static_cast<double>(robot_state_msg.robot_frames(i));
      }

      robot_state.current_errors.joint_position_limits_violation = robot_state_msg.current_errors_joint_position_limits_violation();
      robot_state.current_errors.cartesian_position_limits_violation = robot_state_msg.current_errors_cartesian_position_limits_violation();
      robot_state.current_errors.self_collision_avoidance_violation = robot_state_msg.current_errors_self_collision_avoidance_violation();
      robot_state.current_errors.joint_velocity_violation = robot_state_msg.current_errors_joint_velocity_violation();
      robot_state.current_errors.cartesian_velocity_violation = robot_state_msg.current_errors_cartesian_velocity_violation();
      robot_state.current_errors.force_control_safety_violation = robot_state_msg.current_errors_force_control_safety_violation();
      robot_state.current_errors.joint_reflex = robot_state_msg.current_errors_joint_reflex();
      robot_state.current_errors.cartesian_reflex = robot_state_msg.current_errors_cartesian_reflex();
      robot_state.current_errors.max_goal_pose_deviation_violation = robot_state_msg.current_errors_max_goal_pose_deviation_violation();
      robot_state.current_errors.max_path_pose_deviation_violation = robot_state_msg.current_errors_max_path_pose_deviation_violation();
      robot_state.current_errors.cartesian_velocity_profile_safety_violation = robot_state_msg.current_errors_cartesian_velocity_profile_safety_violation();
      robot_state.current_errors.joint_position_motion_generator_start_pose_invalid = robot_state_msg.current_errors_joint_position_motion_generator_start_pose_invalid();
      robot_state.current_errors.joint_motion_generator_position_limits_violation = robot_state_msg.current_errors_joint_motion_generator_position_limits_violation();
      robot_state.current_errors.joint_motion_generator_velocity_limits_violation = robot_state_msg.current_errors_joint_motion_generator_velocity_limits_violation();
      robot_state.current_errors.joint_motion_generator_velocity_discontinuity = robot_state_msg.current_errors_joint_motion_generator_velocity_discontinuity();
      robot_state.current_errors.joint_motion_generator_acceleration_discontinuity = robot_state_msg.current_errors_joint_motion_generator_acceleration_discontinuity();
      robot_state.current_errors.cartesian_position_motion_generator_start_pose_invalid = robot_state_msg.current_errors_cartesian_position_motion_generator_start_pose_invalid();
      robot_state.current_errors.cartesian_motion_generator_elbow_limit_violation = robot_state_msg.current_errors_cartesian_motion_generator_elbow_limit_violation();
      robot_state.current_errors.cartesian_motion_generator_velocity_limits_violation = robot_state_msg.current_errors_cartesian_motion_generator_velocity_limits_violation();
      robot_state.current_errors.cartesian_motion_generator_velocity_discontinuity = robot_state_msg.current_errors_cartesian_motion_generator_velocity_discontinuity();
      robot_state.current_errors.cartesian_motion_generator_acceleration_discontinuity = robot_state_msg.current_errors_cartesian_motion_generator_acceleration_discontinuity();
      robot_state.current_errors.cartesian_motion_generator_elbow_sign_inconsistent = robot_state_msg.current_errors_cartesian_motion_generator_elbow_sign_inconsistent();
      robot_state.current_errors.cartesian_motion_generator_start_elbow_invalid = robot_state_msg.current_errors_cartesian_motion_generator_start_elbow_invalid();
      robot_state.current_errors.cartesian_motion_generator_joint_position_limits_violation = robot_state_msg.current_errors_cartesian_motion_generator_joint_position_limits_violation();
      robot_state.current_errors.cartesian_motion_generator_joint_velocity_limits_violation = robot_state_msg.current_errors_cartesian_motion_generator_joint_velocity_limits_violation();
      robot_state.current_errors.cartesian_motion_generator_joint_velocity_discontinuity = robot_state_msg.current_errors_cartesian_motion_generator_joint_velocity_discontinuity();
      robot_state.current_errors.cartesian_motion_generator_joint_acceleration_discontinuity = robot_state_msg.current_errors_cartesian_motion_generator_joint_acceleration_discontinuity();
      robot_state.current_errors.cartesian_position_motion_generator_invalid_frame = robot_state_msg.current_errors_cartesian_position_motion_generator_invalid_frame();
      robot_state.current_errors.force_controller_desired_force_tolerance_violation = robot_state_msg.current_errors_force_controller_desired_force_tolerance_violation();
      robot_state.current_errors.controller_torque_discontinuity = robot_state_msg.current_errors_controller_torque_discontinuity();
      robot_state.current_errors.start_elbow_sign_inconsistent = robot_state_msg.current_errors_start_elbow_sign_inconsistent();
      robot_state.current_errors.communication_constraints_violation = robot_state_msg.current_errors_communication_constraints_violation();
      robot_state.current_errors.power_limit_violation = robot_state_msg.current_errors_power_limit_violation();
      robot_state.current_errors.joint_p2p_insufficient_torque_for_planning = robot_state_msg.current_errors_joint_p2p_insufficient_torque_for_planning();
      robot_state.current_errors.tau_j_range_violation = robot_state_msg.current_errors_tau_j_range_violation();
      robot_state.current_errors.instability_detected = robot_state_msg.current_errors_instability_detected();
      robot_state.current_errors.joint_move_in_wrong_direction = robot_state_msg.current_errors_joint_move_in_wrong_direction();

      robot_state.last_motion_errors.joint_position_limits_violation = robot_state_msg.last_motion_errors_joint_position_limits_violation();
      robot_state.last_motion_errors.cartesian_position_limits_violation = robot_state_msg.last_motion_errors_cartesian_position_limits_violation();
      robot_state.last_motion_errors.self_collision_avoidance_violation = robot_state_msg.last_motion_errors_self_collision_avoidance_violation();
      robot_state.last_motion_errors.joint_velocity_violation = robot_state_msg.last_motion_errors_joint_velocity_violation();
      robot_state.last_motion_errors.cartesian_velocity_violation = robot_state_msg.last_motion_errors_cartesian_velocity_violation();
      robot_state.last_motion_errors.force_control_safety_violation = robot_state_msg.last_motion_errors_force_control_safety_violation();
      robot_state.last_motion_errors.joint_reflex = robot_state_msg.last_motion_errors_joint_reflex();
      robot_state.last_motion_errors.cartesian_reflex = robot_state_msg.last_motion_errors_cartesian_reflex();
      robot_state.last_motion_errors.max_goal_pose_deviation_violation = robot_state_msg.last_motion_errors_max_goal_pose_deviation_violation();
      robot_state.last_motion_errors.max_path_pose_deviation_violation = robot_state_msg.last_motion_errors_max_path_pose_deviation_violation();
      robot_state.last_motion_errors.cartesian_velocity_profile_safety_violation = robot_state_msg.last_motion_errors_cartesian_velocity_profile_safety_violation();
      robot_state.last_motion_errors.joint_position_motion_generator_start_pose_invalid = robot_state_msg.last_motion_errors_joint_position_motion_generator_start_pose_invalid();
      robot_state.last_motion_errors.joint_motion_generator_position_limits_violation = robot_state_msg.last_motion_errors_joint_motion_generator_position_limits_violation();
      robot_state.last_motion_errors.joint_motion_generator_velocity_limits_violation = robot_state_msg.last_motion_errors_joint_motion_generator_velocity_limits_violation();
      robot_state.last_motion_errors.joint_motion_generator_velocity_discontinuity = robot_state_msg.last_motion_errors_joint_motion_generator_velocity_discontinuity();
      robot_state.last_motion_errors.joint_motion_generator_acceleration_discontinuity = robot_state_msg.last_motion_errors_joint_motion_generator_acceleration_discontinuity();
      robot_state.last_motion_errors.cartesian_position_motion_generator_start_pose_invalid = robot_state_msg.last_motion_errors_cartesian_position_motion_generator_start_pose_invalid();
      robot_state.last_motion_errors.cartesian_motion_generator_elbow_limit_violation = robot_state_msg.last_motion_errors_cartesian_motion_generator_elbow_limit_violation();
      robot_state.last_motion_errors.cartesian_motion_generator_velocity_limits_violation = robot_state_msg.last_motion_errors_cartesian_motion_generator_velocity_limits_violation();
      robot_state.last_motion_errors.cartesian_motion_generator_velocity_discontinuity = robot_state_msg.last_motion_errors_cartesian_motion_generator_velocity_discontinuity();
      robot_state.last_motion_errors.cartesian_motion_generator_acceleration_discontinuity = robot_state_msg.last_motion_errors_cartesian_motion_generator_acceleration_discontinuity();
      robot_state.last_motion_errors.cartesian_motion_generator_elbow_sign_inconsistent = robot_state_msg.last_motion_errors_cartesian_motion_generator_elbow_sign_inconsistent();
      robot_state.last_motion_errors.cartesian_motion_generator_start_elbow_invalid = robot_state_msg.last_motion_errors_cartesian_motion_generator_start_elbow_invalid();
      robot_state.last_motion_errors.cartesian_motion_generator_joint_position_limits_violation = robot_state_msg.last_motion_errors_cartesian_motion_generator_joint_position_limits_violation();
      robot_state.last_motion_errors.cartesian_motion_generator_joint_velocity_limits_violation = robot_state_msg.last_motion_errors_cartesian_motion_generator_joint_velocity_limits_violation();
      robot_state.last_motion_errors.cartesian_motion_generator_joint_velocity_discontinuity = robot_state_msg.last_motion_errors_cartesian_motion_generator_joint_velocity_discontinuity();
      robot_state.last_motion_errors.cartesian_motion_generator_joint_acceleration_discontinuity = robot_state_msg.last_motion_errors_cartesian_motion_generator_joint_acceleration_discontinuity();
      robot_state.last_motion_errors.cartesian_position_motion_generator_invalid_frame = robot_state_msg.last_motion_errors_cartesian_position_motion_generator_invalid_frame();
      robot_state.last_motion_errors.force_controller_desired_force_tolerance_violation = robot_state_msg.last_motion_errors_force_controller_desired_force_tolerance_violation();
      robot_state.last_motion_errors.controller_torque_discontinuity = robot_state_msg.last_motion_errors_controller_torque_discontinuity();
      robot_state.last_motion_errors.start_elbow_sign_inconsistent = robot_state_msg.last_motion_errors_start_elbow_sign_inconsistent();
      robot_state.last_motion_errors.communication_constraints_violation = robot_state_msg.last_motion_errors_communication_constraints_violation();
      robot_state.last_motion_errors.power_limit_violation = robot_state_msg.last_motion_errors_power_limit_violation();
      robot_state.last_motion_errors.joint_p2p_insufficient_torque_for_planning = robot_state_msg.last_motion_errors_joint_p2p_insufficient_torque_for_planning();
      robot_state.last_motion_errors.tau_j_range_violation = robot_state_msg.last_motion_errors_tau_j_range_violation();
      robot_state.last_motion_errors.instability_detected = robot_state_msg.last_motion_errors_instability_detected();
      robot_state.last_motion_errors.joint_move_in_wrong_direction = robot_state_msg.last_motion_errors_joint_move_in_wrong_direction();
      
      robot_state.control_command_success_rate = robot_state_msg.control_command_success_rate();

      robot_state.robot_mode = static_cast<uint8_t>(robot_state_msg.robot_mode());

      robot_state.robot_time = robot_state_msg.robot_time();

      robot_state.gripper_width = robot_state_msg.gripper_width();

      robot_state.gripper_max_width = robot_state_msg.gripper_max_width();

      robot_state.gripper_is_grasped = robot_state_msg.gripper_is_grasped();

      robot_state.gripper_temperature = static_cast<uint16_t>(robot_state_msg.gripper_temperature());

      robot_state.gripper_time = robot_state_msg.gripper_time();

      robot_state.is_fresh = true;
    } else {
      robot_state.is_fresh = false;
    }

    return robot_state;
  }

  franka_interface_msgs::FrankaInterfaceStatus SharedMemoryHandler::getFrankaInterfaceStatus()
  {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> franka_interface_state_info_lock(*franka_interface_state_info_mutex_, boost::interprocess::defer_lock);

    franka_interface_msgs::FrankaInterfaceStatus franka_interface_status;
    franka_interface_status.header.stamp = ros::Time::now();

    if (franka_interface_state_info_lock.try_lock()) {
      // TODO(jacky): 25 roughly equates to 500ms of disconnect from the franka_interface. This should be placed in a global config file.
      if (franka_interface_state_info_->get_watchdog_counter() > 25) {
        franka_interface_status.is_ready = false;
      } else {
        franka_interface_status.is_ready = franka_interface_state_info_->get_is_ready();
      }
      franka_interface_status.error_description = franka_interface_state_info_->get_error_description();
      franka_interface_status.is_fresh = true;
    } else {
      franka_interface_status.is_fresh = false;
    }
    
    return franka_interface_status;
  }

  franka_interface_msgs::RunLoopProcessInfoState SharedMemoryHandler::getRunLoopProcessInfoState()
  {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_, boost::interprocess::defer_lock);

    franka_interface_msgs::RunLoopProcessInfoState run_loop_process_info_state;
    run_loop_process_info_state.header.stamp = ros::Time::now();

    if (run_loop_info_lock.try_lock()) {
      run_loop_process_info_state.current_memory_region = run_loop_process_info_->get_current_memory_region();
      run_loop_process_info_state.current_sensor_region = run_loop_process_info_->get_current_sensor_region();
      run_loop_process_info_state.current_feedback_region = run_loop_process_info_->get_current_feedback_region();
      run_loop_process_info_state.current_skill_id = run_loop_process_info_->get_current_skill_id();
      run_loop_process_info_state.current_skill_type = run_loop_process_info_->get_current_skill_type();
      run_loop_process_info_state.current_meta_skill_id = run_loop_process_info_->get_current_meta_skill_id();
      run_loop_process_info_state.current_meta_skill_type = run_loop_process_info_->get_current_meta_skill_type();
      run_loop_process_info_state.current_skill_description = run_loop_process_info_->get_current_skill_description();
      run_loop_process_info_state.new_skill_available = run_loop_process_info_->get_new_skill_available();
      run_loop_process_info_state.new_skill_id = run_loop_process_info_->get_new_skill_id();
      run_loop_process_info_state.new_skill_type = run_loop_process_info_->get_new_skill_type();
      run_loop_process_info_state.new_meta_skill_id = run_loop_process_info_->get_new_meta_skill_id();
      run_loop_process_info_state.new_meta_skill_type = run_loop_process_info_->get_new_meta_skill_type();
      run_loop_process_info_state.new_skill_description = run_loop_process_info_->get_new_skill_description();
      run_loop_process_info_state.is_running_skill = run_loop_process_info_->get_is_running_skill();
      run_loop_process_info_state.skill_preempted = run_loop_process_info_->get_skill_preempted();
      run_loop_process_info_state.done_skill_id = run_loop_process_info_->get_done_skill_id();
      run_loop_process_info_state.result_skill_id = run_loop_process_info_->get_result_skill_id();
      run_loop_process_info_state.time_skill_started_in_robot_time = run_loop_process_info_->get_time_skill_started_in_robot_time();
      run_loop_process_info_state.time_skill_finished_in_robot_time = run_loop_process_info_->get_time_skill_finished_in_robot_time();
      
      run_loop_process_info_state.is_fresh = true;
    } else {
      run_loop_process_info_state.is_fresh = false;
    }
    
    return run_loop_process_info_state;
  }

  void SharedMemoryHandler::incrementWatchdogCounter()
  {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> franka_interface_state_info_lock(*franka_interface_state_info_mutex_, boost::interprocess::defer_lock);
    if (franka_interface_state_info_lock.try_lock()) {
      franka_interface_state_info_->increment_watchdog_counter();
    }
  }
  
  bool SharedMemoryHandler::getNewSkillAvailableFlagInSharedMemory()
  {
    // Grab the lock of the run_loop_info_mutex_
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_);

    // Return the done_skill_id
    return getNewSkillAvailableFlagInSharedMemoryUnprotected();

    // The lock of the run_loop_info_mutex_ should be released automatically
  }

  int SharedMemoryHandler::getNewSkillIdInSharedMemory()
  {
    // Grab the lock of the run_loop_info_mutex_
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(*run_loop_info_mutex_);

    // Return the done_skill_id
    return getNewSkillIdInSharedMemoryUnprotected();

    // The lock of the run_loop_info_mutex_ should be released automatically
  }

  // ALL UNPROTECTED FUNCTIONS BELOW REQUIRE A MUTEX OVER THE RUN_LOOP_INFO

  int SharedMemoryHandler::getCurrentFreeSharedMemoryIndexInSharedMemoryUnprotected()
  {
    // Return the current_free_shared_memory_index from run_loop_process_info_
    return run_loop_process_info_->get_current_free_shared_memory_index();
  }

  int SharedMemoryHandler::getCurrentSkillIdInSharedMemoryUnprotected()
  {
    // Return the current_skill_id from run_loop_process_info_
    return run_loop_process_info_->get_current_skill_id();
  }

  int SharedMemoryHandler::getDoneSkillIdInSharedMemoryUnprotected()
  {
    // Return the done_skill_id from run_loop_process_info_
    return run_loop_process_info_->get_done_skill_id();
  }

  bool SharedMemoryHandler::getNewSkillAvailableFlagInSharedMemoryUnprotected()
  {
    // Return the new_skill_available_flag
    return run_loop_process_info_->get_new_skill_available();
  }

  void SharedMemoryHandler::setNewSkillFlagInSharedMemoryUnprotected(bool new_skill_flag)
  {
    // Set new_skill_available_ in run_loop_process_info_ to the input new_skill_flag
    run_loop_process_info_->set_new_skill_available(new_skill_flag);
  }

  int SharedMemoryHandler::getNewSkillIdInSharedMemoryUnprotected()
  {
    // Return the new_skill_id from run_loop_process_info_
    return run_loop_process_info_->get_new_skill_id();
  }

  void SharedMemoryHandler::setNewSkillIdInSharedMemoryUnprotected(int new_skill_id)
  {
    // Set new_skill_id_ in run_loop_process_info_ to the input new_skill_id
    run_loop_process_info_->set_new_skill_id(new_skill_id);
  }

  void SharedMemoryHandler::setNewSkillDescriptionInSharedMemoryUnprotected(std::string description)
  {
    // Set new_skill_id_ in run_loop_process_info_ to the input new_skill_id
    run_loop_process_info_->set_new_skill_description(description);
  }

  void SharedMemoryHandler::setNewSkillTypeInSharedMemoryUnprotected(int new_skill_type)
  {
    // Set new_skill_type_ in run_loop_process_info_ to the input new_skill_type
    run_loop_process_info_->set_new_skill_type(new_skill_type);
  }

  void SharedMemoryHandler::setNewMetaSkillIdInSharedMemoryUnprotected(int new_meta_skill_id)
  {
    // Set new_meta_skill_id_ in run_loop_process_info_ to the input new_meta_skill_id
    run_loop_process_info_->set_new_meta_skill_id(new_meta_skill_id);
  }

  void SharedMemoryHandler::setNewMetaSkillTypeInSharedMemoryUnprotected(int new_meta_skill_type)
  {
    // Set new_meta_skill_type_ in run_loop_process_info_ to the input new_meta_skill_type
    run_loop_process_info_->set_new_meta_skill_type(new_meta_skill_type);
  }

  void SharedMemoryHandler::setResultSkillIdInSharedMemoryUnprotected(int result_skill_id)
  {
    // Set result_skill_id_ in run_loop_process_info_ to the input result_skill_id
    run_loop_process_info_->set_result_skill_id(result_skill_id);
  }

  // Loads sensor data into the designated sensor memory buffer
  // Requires a lock on the mutex of the designated sensor memory buffer
  void SharedMemoryHandler::loadSensorDataUnprotected(const franka_interface_msgs::ExecuteSkillGoalConstPtr &goal,
                                                      int current_free_shared_memory_index) {
    // Do nothing.
  }

  // WARNING: THIS ASSUMES MUTEX HAS ALREADY BEEN OBTAINED
  void SharedMemoryHandler::loadSensorDataIntoSharedMemory(const franka_interface_msgs::SensorData &sensor_data, SensorBufferTypePtr sensor_buffer_ptr) {
    int sensor_data_size = sensor_data.size;

    // First let's indicate this is new data.
    sensor_buffer_ptr[0] = 1;
    // Now add the type for the message.
    sensor_buffer_ptr[1] = sensor_data.type;
    // Now add the size of the data.
    sensor_buffer_ptr[2] = (sensor_data_size & 0xFF);
    sensor_buffer_ptr[3] = ((sensor_data_size >> 8) & 0xFF);
    sensor_buffer_ptr[4] = ((sensor_data_size >> 16) & 0xFF);
    sensor_buffer_ptr[5] = ((sensor_data_size >> 24) & 0xFF);

    memcpy(sensor_buffer_ptr + 6, &sensor_data.sensorData[0], sensor_data_size * sizeof(uint8_t));
  }

  //Adding new function to load sensor data into sensor memory buffer
  void SharedMemoryHandler::tryToLoadSensorDataGroupIntoSharedMemory(const franka_interface_msgs::SensorDataGroup::ConstPtr &sensor_data_group_ptr) {
    if (sensor_data_group_0_mutex_->try_lock())
    {
      if (sensor_data_group_ptr->has_trajectory_generator_sensor_data) {
        loadSensorDataIntoSharedMemory(sensor_data_group_ptr->trajectoryGeneratorSensorData, sensor_data_trajectory_generator_buffer_0_);
      }

      if (sensor_data_group_ptr->has_feedback_controller_sensor_data) {
        loadSensorDataIntoSharedMemory(sensor_data_group_ptr->feedbackControllerSensorData, sensor_data_feedback_controller_buffer_0_);
      }

      if (sensor_data_group_ptr->has_termination_handler_sensor_data) {
        loadSensorDataIntoSharedMemory(sensor_data_group_ptr->terminationHandlerSensorData, sensor_data_termination_handler_buffer_0_);
      }

      sensor_data_group_0_mutex_->unlock();
    } else {
      ROS_INFO("Failed to get sensor data group 0 mutex");
    }
  }

  // Loads traj gen parameters into the designated current_free_shared_memory_index buffer
  // Requires a lock on the mutex of the designated current_free_shared_memory_index buffer
  void SharedMemoryHandler::loadTrajGenParamsUnprotected(const franka_interface_msgs::ExecuteSkillGoalConstPtr &goal,
                                                         int current_free_shared_memory_index)
  {
    int trajectory_generator_param_data_size = goal->trajectory_generator_param_data_size;

    if(current_free_shared_memory_index == 0)
    {
      // First is the type of the trajectory generator
      trajectory_generator_buffer_0_[0] = static_cast<SharedBufferType>(goal->trajectory_generator_type);
      // Second is the size of the data
      trajectory_generator_buffer_0_[1] = (trajectory_generator_param_data_size & 0xFF);
      trajectory_generator_buffer_0_[2] = ((trajectory_generator_param_data_size >> 8) & 0xFF);
      trajectory_generator_buffer_0_[3] = ((trajectory_generator_param_data_size >> 16) & 0xFF);
      trajectory_generator_buffer_0_[4] = ((trajectory_generator_param_data_size >> 24) & 0xFF);
      // Now mem copy all of the data in the form of bytes
      memcpy(trajectory_generator_buffer_0_ + 5, &goal->trajectory_generator_param_data[0], 
             trajectory_generator_param_data_size * sizeof(uint8_t));
    }
    else if(current_free_shared_memory_index == 1)
    {
      // First is the type of the trajectory generator
      trajectory_generator_buffer_1_[0] = static_cast<SharedBufferType>(goal->trajectory_generator_type);
      // Second is the size of the data
      trajectory_generator_buffer_1_[1] = (trajectory_generator_param_data_size & 0xFF);
      trajectory_generator_buffer_1_[2] = ((trajectory_generator_param_data_size >> 8) & 0xFF);
      trajectory_generator_buffer_1_[3] = ((trajectory_generator_param_data_size >> 16) & 0xFF);
      trajectory_generator_buffer_1_[4] = ((trajectory_generator_param_data_size >> 24) & 0xFF);
      // Now mem copy all of the data in the form of bytes
      memcpy(trajectory_generator_buffer_1_ + 5, &goal->trajectory_generator_param_data[0], 
             trajectory_generator_param_data_size * sizeof(uint8_t));
    }
  }

  // Loads feedback controller parameters into the designated current_free_shared_memory_index buffer
  // Requires a lock on the mutex of the designated current_free_shared_memory_index buffer
  void SharedMemoryHandler::loadFeedbackControllerParamsUnprotected(
      const franka_interface_msgs::ExecuteSkillGoalConstPtr &goal, int current_free_shared_memory_index)
  {
    int feedback_controller_param_data_size = goal->feedback_controller_param_data_size;

    if(current_free_shared_memory_index == 0)
    {
      // First is the type of the trajectory generator
      feedback_controller_buffer_0_[0] = static_cast<SharedBufferType>(goal->feedback_controller_type);
      // Second is the size of the data
      feedback_controller_buffer_0_[1] = (feedback_controller_param_data_size & 0xFF);
      feedback_controller_buffer_0_[2] = ((feedback_controller_param_data_size >> 8) & 0xFF);
      feedback_controller_buffer_0_[3] = ((feedback_controller_param_data_size >> 16) & 0xFF);
      feedback_controller_buffer_0_[4] = ((feedback_controller_param_data_size >> 24) & 0xFF);
      // Now mem copy all of the data in the form of bytes
      memcpy(feedback_controller_buffer_0_ + 5, &goal->feedback_controller_param_data[0], 
             feedback_controller_param_data_size * sizeof(uint8_t));
    }
    else if(current_free_shared_memory_index == 1)
    {
      // First is the type of the trajectory generator
      feedback_controller_buffer_1_[0] = static_cast<SharedBufferType>(goal->feedback_controller_type);
      // Second is the size of the data
      feedback_controller_buffer_1_[1] = (feedback_controller_param_data_size & 0xFF);
      feedback_controller_buffer_1_[2] = ((feedback_controller_param_data_size >> 8) & 0xFF);
      feedback_controller_buffer_1_[3] = ((feedback_controller_param_data_size >> 16) & 0xFF);
      feedback_controller_buffer_1_[4] = ((feedback_controller_param_data_size >> 24) & 0xFF);
      // Now mem copy all of the data in the form of bytes
      memcpy(feedback_controller_buffer_1_ + 5, &goal->feedback_controller_param_data[0], 
             feedback_controller_param_data_size * sizeof(uint8_t));
    }
  }

  // Loads termination parameters into the designated current_free_shared_memory_index buffer
  // Requires a lock on the mutex of the designated current_free_shared_memory_index buffer
  void SharedMemoryHandler::loadTerminationParamsUnprotected(const franka_interface_msgs::ExecuteSkillGoalConstPtr &goal,
                                                             int current_free_shared_memory_index)
  {
    int termination_handler_param_data_size = goal->termination_handler_param_data_size;

    if(current_free_shared_memory_index == 0)
    {
      // First is the type of the trajectory generator
      termination_handler_buffer_0_[0] = static_cast<SharedBufferType>(goal->termination_handler_type);
      // Second is the size of the data
      termination_handler_buffer_0_[1] = (termination_handler_param_data_size & 0xFF);
      termination_handler_buffer_0_[2] = ((termination_handler_param_data_size >> 8) & 0xFF);
      termination_handler_buffer_0_[3] = ((termination_handler_param_data_size >> 16) & 0xFF);
      termination_handler_buffer_0_[4] = ((termination_handler_param_data_size >> 24) & 0xFF);
      // Now mem copy all of the data in the form of bytes
      memcpy(termination_handler_buffer_0_ + 5, &goal->termination_handler_param_data[0], 
             termination_handler_param_data_size * sizeof(uint8_t));
    }
    else if(current_free_shared_memory_index == 1)
    {
      // First is the type of the trajectory generator
      termination_handler_buffer_1_[0] = static_cast<SharedBufferType>(goal->termination_handler_type);
      // Second is the size of the data
      termination_handler_buffer_1_[1] = (termination_handler_param_data_size & 0xFF);
      termination_handler_buffer_1_[2] = ((termination_handler_param_data_size >> 8) & 0xFF);
      termination_handler_buffer_1_[3] = ((termination_handler_param_data_size >> 16) & 0xFF);
      termination_handler_buffer_1_[4] = ((termination_handler_param_data_size >> 24) & 0xFF);
      // Now mem copy all of the data in the form of bytes
      memcpy(termination_handler_buffer_1_ + 5, &goal->termination_handler_param_data[0], 
             termination_handler_param_data_size * sizeof(uint8_t));
    }
  }

  // Loads timer parameters into the designated current_free_shared_memory_index buffer
  // Requires a lock on the mutex of the designated current_free_shared_memory_index buffer
  void SharedMemoryHandler::loadTimerParamsUnprotected(const franka_interface_msgs::ExecuteSkillGoalConstPtr &goal,
                                                       int current_free_shared_memory_index)
  {
    if(current_free_shared_memory_index == 0)
    {
      timer_buffer_0_[0] = static_cast<SharedBufferType>(goal->timer_type);
      timer_buffer_0_[1] = static_cast<SharedBufferType>(goal->num_timer_params);
      memcpy(timer_buffer_0_ + 2, &goal->timer_params[0], goal->num_timer_params * sizeof(SharedBufferType));
    }
    else if(current_free_shared_memory_index == 1)
    {
      timer_buffer_1_[0] = static_cast<SharedBufferType>(goal->timer_type);
      timer_buffer_1_[1] = static_cast<SharedBufferType>(goal->num_timer_params);
      memcpy(timer_buffer_1_ + 2, &goal->timer_params[0], goal->num_timer_params * sizeof(SharedBufferType));
    }
  }

}
