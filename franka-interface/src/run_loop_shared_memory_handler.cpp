//
// Created by mohit on 12/18/18.
//

#include "franka-interface/run_loop_shared_memory_handler.h"

#include <iostream>

RunLoopProcessInfo* RunLoopSharedMemoryHandler::getRunLoopProcessInfo() {
  return run_loop_info_;
}

FrankaInterfaceStateInfo* RunLoopSharedMemoryHandler::getFrankaInterfaceStateInfo() {
  return franka_interface_state_info_;
}

boost::interprocess::interprocess_mutex* RunLoopSharedMemoryHandler::getRunLoopProcessInfoMutex() {
  return run_loop_info_mutex_;
}

boost::interprocess::interprocess_mutex* RunLoopSharedMemoryHandler::getFrankaInterfaceStateInfoMutex() {
  return franka_interface_state_info_mutex_;
}

boost::interprocess::interprocess_mutex* RunLoopSharedMemoryHandler::getCurrentRobotStateBufferMutex() {
  return shared_current_robot_state_mutex_;
}

boost::interprocess::interprocess_mutex* RunLoopSharedMemoryHandler::getSensorDataGroupBufferMutex() {
    return sensor_data_group_mutex_0_;
}

SharedBufferTypePtr RunLoopSharedMemoryHandler::getTrajectoryGeneratorBuffer(int memory_region) {
  if (memory_region == 0) {
    return traj_gen_buffer_0_;
  } else if (memory_region == 1) {
    return traj_gen_buffer_1_;
  } else {
    std::cout << "Incorrect memory region for trajectory generator\n";
    return nullptr;
  }
}

SharedBufferTypePtr RunLoopSharedMemoryHandler::getFeedbackControllerBuffer(int memory_region) {
  if (memory_region == 0) {
    return feedback_controller_buffer_0_;
  } else if (memory_region == 1) {
    return feedback_controller_buffer_1_;
  } else {
    std::cout << "Incorrect memory region for feedback controller\n";
    return nullptr;
  }
}

SharedBufferTypePtr RunLoopSharedMemoryHandler::getTerminationParametersBuffer(int memory_region) {
  if (memory_region == 0) {
    return termination_buffer_0_;
  } else if (memory_region == 1) {
    return termination_buffer_1_;
  } else {
    std::cout << "Incorrect memory region for termination parameters\n";
    return nullptr;
  }
}

SharedBufferTypePtr RunLoopSharedMemoryHandler::getExecutionResultBuffer(int memory_region) {
  if (memory_region == 0) {
    return execution_result_buffer_0_;
  } else if (memory_region == 1) {
    return execution_result_buffer_1_;
  } else {
    std::cout << "Incorrect memory region for execution result buffer\n";
    return nullptr;
  }
}

SharedBufferTypePtr RunLoopSharedMemoryHandler::getFeedbackResultBuffer(int memory_region) {
  if (memory_region == 0) {
    return execution_feedback_buffer_0_;
  } else if (memory_region == 1) {
    return execution_feedback_buffer_1_;
  } else {
    std::cout << "Incorrect memory region for execution feedback result buffer\n";
    return nullptr;
  }
}

SharedBufferTypePtr RunLoopSharedMemoryHandler::getCurrentRobotStateBuffer() {
    return current_robot_state_buffer_;
}

SensorBufferTypePtr RunLoopSharedMemoryHandler::getSensorDataTrajectoryGeneratorBuffer(int memory_region) {
    if (memory_region == 0) {
        return sensor_data_trajectory_generator_buffer_0_;
    } else {
        std::cout << "Incorrect memory region for sensor data\n";
        return nullptr;
    }
}

SensorBufferTypePtr RunLoopSharedMemoryHandler::getSensorDataFeedbackControllerBuffer(int memory_region) {
    if (memory_region == 0) {
        return sensor_data_feedback_controller_buffer_0_;
    } else {
        std::cout << "Incorrect memory region for sensor data\n";
        return nullptr;
    }
}

SensorBufferTypePtr RunLoopSharedMemoryHandler::getSensorDataTerminationHandlerBuffer(int memory_region) {
    if (memory_region == 0) {
        return sensor_data_termination_handler_buffer_0_;
    } else {
        std::cout << "Incorrect memory region for sensor data\n";
        return nullptr;
    }
}


void RunLoopSharedMemoryHandler::clearAllBuffers() {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> shared_memory_0_lock(*shared_memory_mutex_0_);
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> shared_memory_1_lock(*shared_memory_mutex_1_);
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> shared_execution_result_0_lock(*shared_execution_result_mutex_0_);
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> shared_execution_result_1_lock(*shared_execution_result_mutex_1_);
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> shared_current_robot_state_lock(*shared_current_robot_state_mutex_);

    memset(traj_gen_buffer_0_, 0.0, shared_memory_info_.getSizeForTrajectoryParameters());
    memset(feedback_controller_buffer_0_, 0.0, shared_memory_info_.getSizeForFeedbackControllerParameters());
    memset(termination_buffer_0_, 0.0, shared_memory_info_.getSizeForTerminationParameters());
    memset(timer_buffer_0_, 0.0, shared_memory_info_.getSizeForTimerParameters());

    // sensor_data_buffer is of type uint8_t
    memset(sensor_data_trajectory_generator_buffer_0_, 0, shared_memory_info_.getSizeForSensorData());
    memset(sensor_data_feedback_controller_buffer_0_, 0, shared_memory_info_.getSizeForSensorData());
    memset(sensor_data_termination_handler_buffer_0_, 0, shared_memory_info_.getSizeForSensorData());

    memset(traj_gen_buffer_1_, 0.0, shared_memory_info_.getSizeForTrajectoryParameters());
    memset(feedback_controller_buffer_1_, 0.0, shared_memory_info_.getSizeForFeedbackControllerParameters());
    memset(termination_buffer_1_, 0.0, shared_memory_info_.getSizeForTerminationParameters());
    memset(timer_buffer_1_, 0.0, shared_memory_info_.getSizeForTimerParameters());

    memset(execution_result_buffer_0_, 0.0, shared_memory_info_.getSizeForExecutionResultData());
    memset(execution_feedback_buffer_0_, 0.0, shared_memory_info_.getSizeForExecutionFeedbackData());

    memset(execution_result_buffer_1_, 0.0, shared_memory_info_.getSizeForExecutionResultData());
    memset(execution_feedback_buffer_1_, 0.0, shared_memory_info_.getSizeForExecutionFeedbackData());

    memset(current_robot_state_buffer_, 0.0, shared_memory_info_.getSizeForCurrentRobotState());
}

void RunLoopSharedMemoryHandler::start() {
  // Start processing, might want to do some pre-processing
  std::cout << "start run loop.\n";

  // Create managed shared memory (segments) here.
  boost::interprocess::shared_memory_object::remove(shared_memory_info_.getSharedMemoryNameForObjects().c_str());
  managed_shared_memory_ = boost::interprocess::managed_shared_memory(
      boost::interprocess::create_only,
      shared_memory_info_.getSharedMemoryNameForObjects().c_str(),
      shared_memory_info_.getObjectMemorySize());

  // Add run loop process info to the main loop.
  run_loop_info_ = managed_shared_memory_.construct<RunLoopProcessInfo>
      (shared_memory_info_.getRunLoopInfoObjectName().c_str())
      (1);

  // Add the inter-process mutex into memory. We will grab this each
  // time we want to update anything in the memory.
  run_loop_info_mutex_ = managed_shared_memory_.construct<
      boost::interprocess::interprocess_mutex>
      (shared_memory_info_.getRunLoopInfoMutexName().c_str())
      ();

  // Add franka interface state info to the main loop.
  franka_interface_state_info_ = managed_shared_memory_.construct<FrankaInterfaceStateInfo>
      (shared_memory_info_.getFrankaInterfaceStateInfoObjectName().c_str())
      ();

  // Add the inter-process mutex into memory. We will grab this each
  // time we want to update anything in the memory.
  franka_interface_state_info_mutex_ = managed_shared_memory_.construct<
      boost::interprocess::interprocess_mutex>
      (shared_memory_info_.getFrankaInterfaceStateInfoMutexName().c_str())
      ();

  /**
   * Create shared memory region for buffer 0.
   */
  const char *shm_name_0 = shared_memory_info_.getSharedMemoryNameForParameters(0).c_str();
  boost::interprocess::shared_memory_object::remove(shm_name_0);
  shared_memory_object_0_ = boost::interprocess::shared_memory_object(
      boost::interprocess::open_or_create,
      shared_memory_info_.getSharedMemoryNameForParameters(0).c_str(),
      boost::interprocess::read_write
  );
  shared_memory_object_0_.truncate(shared_memory_info_.getParameterMemorySize(0));

  region_traj_params_0_=  boost::interprocess::mapped_region(
      shared_memory_object_0_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForTrajectoryParameters(),
      shared_memory_info_.getSizeForTrajectoryParameters()
  );
  traj_gen_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(region_traj_params_0_.get_address());
  region_feedback_controller_params_0_ = boost::interprocess::mapped_region(
      shared_memory_object_0_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForFeedbackControllerParameters(),
      shared_memory_info_.getSizeForFeedbackControllerParameters()
  );
  feedback_controller_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>
  (region_feedback_controller_params_0_.get_address());
  region_termination_params_0_ = boost::interprocess::mapped_region(
      shared_memory_object_0_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForTerminationParameters(),
      shared_memory_info_.getSizeForTerminationParameters()
  );
  termination_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(
      region_termination_params_0_.get_address());
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

  sensor_data_trajectory_generator_buffer_0_ = reinterpret_cast<SensorBufferTypePtr >(region_sensor_data_trajectory_generator_0_.get_address());

  region_sensor_data_feedback_controller_0_ =  boost::interprocess::mapped_region(
            shared_memory_object_0_,
            boost::interprocess::read_write,
            shared_memory_info_.getOffsetForSensorDataFeedbackController(),
            shared_memory_info_.getSizeForSensorData()
  );

  sensor_data_feedback_controller_buffer_0_ = reinterpret_cast<SensorBufferTypePtr >(region_sensor_data_feedback_controller_0_.get_address());

  region_sensor_data_termination_handler_0_ =  boost::interprocess::mapped_region(
            shared_memory_object_0_,
            boost::interprocess::read_write,
            shared_memory_info_.getOffsetForSensorDataTerminationHandler(),
            shared_memory_info_.getSizeForSensorData()
  );

  sensor_data_termination_handler_buffer_0_ = reinterpret_cast<SensorBufferTypePtr >(region_sensor_data_termination_handler_0_.get_address());

  /**
   * Create shared memory region for buffer 1.
   */

  // Create shared memory objects. for different parameters
  const char *shm_name_1 = shared_memory_info_.getSharedMemoryNameForParameters(1).c_str();
  boost::interprocess::shared_memory_object::remove(shm_name_1);
  shared_memory_object_1_ = boost::interprocess::shared_memory_object(
      boost::interprocess::open_or_create,
      shared_memory_info_.getSharedMemoryNameForParameters(1).c_str(),
      boost::interprocess::read_write
  );

  // Allocate memory
  shared_memory_object_1_.truncate(shared_memory_info_.getParameterMemorySize(1));

  // Allocate regions for each parameter array
  region_traj_params_1_ =  boost::interprocess::mapped_region(
      shared_memory_object_1_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForTrajectoryParameters(),
      shared_memory_info_.getSizeForTrajectoryParameters()
  );
  traj_gen_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(region_traj_params_1_.get_address());
  region_feedback_controller_params_1_ = boost::interprocess::mapped_region(
      shared_memory_object_1_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForFeedbackControllerParameters(),
      shared_memory_info_.getSizeForFeedbackControllerParameters()
  );
  feedback_controller_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>
  (region_feedback_controller_params_1_.get_address());
  region_termination_params_1_ = boost::interprocess::mapped_region(
      shared_memory_object_1_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForTerminationParameters(),
      shared_memory_info_.getSizeForTerminationParameters()
  );
  termination_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(
      region_termination_params_1_.get_address());
  region_timer_params_1_ = boost::interprocess::mapped_region(
      shared_memory_object_1_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForTimerParameters(),
      shared_memory_info_.getSizeForTimerParameters()
  );
  timer_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(region_timer_params_1_.get_address());

  /**
   * Create mutexes for parameter buffers.
   */
  shared_memory_mutex_0_ = managed_shared_memory_.construct<
      boost::interprocess::interprocess_mutex>
      (shared_memory_info_.getParameterMemoryMutexName(0).c_str())
      ();
  shared_memory_mutex_1_ = managed_shared_memory_.construct<
      boost::interprocess::interprocess_mutex>
      (shared_memory_info_.getParameterMemoryMutexName(1).c_str())
      ();

  /**
   * Create mutexes for sensor data.
   */
  sensor_data_group_mutex_0_ = managed_shared_memory_.construct<
      boost::interprocess::interprocess_mutex>
      (shared_memory_info_.getSensorDataGroupMutexName(0).c_str())
      ();



  /**
   * Create memory 0 for execution response.
   */
  const char *results_0 = shared_memory_info_.getSharedMemoryNameForResults(0).c_str();
  boost::interprocess::shared_memory_object::remove(results_0);
  shared_execution_result_0_ = boost::interprocess::shared_memory_object(
      boost::interprocess::open_or_create,
      shared_memory_info_.getSharedMemoryNameForResults(0).c_str(),
      boost::interprocess::read_write
  );
  shared_execution_result_0_.truncate(shared_memory_info_.getExecutionResponseMemorySize());
  region_execution_feedback_buffer_0 =  boost::interprocess::mapped_region(
      shared_execution_result_0_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForExecutionFeedbackData(),
      shared_memory_info_.getSizeForExecutionFeedbackData()
  );
  execution_feedback_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(region_execution_feedback_buffer_0.get_address());
  region_execution_result_buffer_0_ = boost::interprocess::mapped_region(
      shared_execution_result_0_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForExecutionResultData(),
      shared_memory_info_.getSizeForExecutionResultData()
  );
  execution_result_buffer_0_ = reinterpret_cast<SharedBufferTypePtr>(region_execution_result_buffer_0_.get_address());

  /**
   * Create memory 1 for execution response.
   */
  const char *results_1 = shared_memory_info_.getSharedMemoryNameForResults(1).c_str();
  boost::interprocess::shared_memory_object::remove(results_1);
  shared_execution_result_1_ = boost::interprocess::shared_memory_object(
      boost::interprocess::open_or_create,
      shared_memory_info_.getSharedMemoryNameForResults(1).c_str(),
      boost::interprocess::read_write
  );
  shared_execution_result_1_.truncate(shared_memory_info_.getExecutionResponseMemorySize());
  region_execution_feedback_buffer_1_ =  boost::interprocess::mapped_region(
      shared_execution_result_1_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForExecutionFeedbackData(),
      shared_memory_info_.getSizeForExecutionFeedbackData()
  );
  execution_feedback_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(region_execution_feedback_buffer_1_.get_address());
  region_execution_result_buffer_1_ = boost::interprocess::mapped_region(
      shared_execution_result_1_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForExecutionResultData(),
      shared_memory_info_.getSizeForExecutionResultData()
  );
  execution_result_buffer_1_ = reinterpret_cast<SharedBufferTypePtr>(region_execution_result_buffer_1_.get_address());

      /**
   * Create mutexes for execution response.
   */
  shared_execution_result_mutex_0_ = managed_shared_memory_.construct<
      boost::interprocess::interprocess_mutex>
      (shared_memory_info_.getExecutionResponseMutexName(0).c_str())
      ();
  shared_execution_result_mutex_1_ = managed_shared_memory_.construct<
      boost::interprocess::interprocess_mutex>
      (shared_memory_info_.getExecutionResponseMutexName(1).c_str())
      ();


  /**
   * Create memory for current robot state.
   */
  const char *current_robot_state = shared_memory_info_.getSharedMemoryNameForCurrentRobotState().c_str();
  boost::interprocess::shared_memory_object::remove(current_robot_state);
  shared_current_robot_state_ = boost::interprocess::shared_memory_object(
      boost::interprocess::open_or_create,
      shared_memory_info_.getSharedMemoryNameForCurrentRobotState().c_str(),
      boost::interprocess::read_write
  );
  shared_current_robot_state_.truncate(shared_memory_info_.getCurrentRobotStateMemorySize());
  region_current_robot_state_buffer_ = boost::interprocess::mapped_region(
      shared_current_robot_state_,
      boost::interprocess::read_write,
      shared_memory_info_.getOffsetForCurrentRobotState(),
      shared_memory_info_.getSizeForCurrentRobotState()
  );
  current_robot_state_buffer_ = reinterpret_cast<SharedBufferTypePtr>(region_current_robot_state_buffer_.get_address());

  shared_current_robot_state_mutex_ = managed_shared_memory_.construct<
      boost::interprocess::interprocess_mutex>
      (shared_memory_info_.getCurrentRobotStateMutexName().c_str())
      ();

  std::cout << "Did create all shared memory buffers." << std::endl;
}
