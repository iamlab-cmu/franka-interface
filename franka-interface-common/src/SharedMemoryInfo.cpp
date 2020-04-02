//
// Created by mohit on 11/25/18.
//

#include <franka-interface-common/SharedMemoryInfo.h>

#include <cassert>

SharedMemoryInfo::SharedMemoryInfo() {
  // pass
}


std::string SharedMemoryInfo::getSharedMemoryNameForParameters(int index) {
  if (index == 0) {
    return params_memory_name_0_;
  } else if (index == 1) {
    return params_memory_name_1_;
  } else {
    assert(false);
    return "";
  }
}

std::string SharedMemoryInfo::getSharedMemoryNameForObjects() {
  return objects_memory_name_;
}



std::string SharedMemoryInfo::getSharedMemoryNameForSensorData(int index) {
  if (index == 0) {
    return sensor_data_memory_name_0_;
  } else if (index == 1) {
    return sensor_data_memory_name_1_;
  } else {
    assert(false);
    return "";
  }
}

std::string SharedMemoryInfo::getSharedMemoryNameForResults(int index) {
  if (index == 0) {
    return execution_response_name_0_;
  } else if (index == 1) {
    return execution_response_name_1_;
  } else {
    assert(false);
    return "";
  }
}

std::string SharedMemoryInfo::getSharedMemoryNameForCurrentRobotState() {
  return current_robot_state_name_;
}

std::string SharedMemoryInfo::getRunLoopInfoObjectName() {
  return run_loop_info_name_;
}

std::string SharedMemoryInfo::getRunLoopInfoMutexName() {
  return run_loop_info_mutex_name_;
}

std::string SharedMemoryInfo::getFrankaInterfaceStateInfoObjectName() {
  return franka_interface_state_info_name_;
}

std::string SharedMemoryInfo::getFrankaInterfaceStateInfoMutexName() {
  return franka_interface_state_info_mutex_name_;
}

std::string SharedMemoryInfo::getParameterMemoryMutexName(int index) {
  if (index == 0) {
    return params_memory_mutex_name_0_;
  } else if (index == 1) {
    return params_memory_mutex_name_1_;
  } else {
    assert(false);
    return "";
  }
}

std::string SharedMemoryInfo::getSensorDataGroupMutexName(int index) {
  if (index == 0) {
    return sensor_data_group_mutex_name_0_;
  } else {
    assert(false);
    return "";
  }
}

std::string SharedMemoryInfo::getExecutionResponseMutexName(int index) {
  if (index == 0) {
    return execution_response_mutex_name_0_;
  } else if (index == 1) {
    return execution_response_mutex_name_1_;
  } else {
    assert(false);
    return "";
  }
}

std::string SharedMemoryInfo::getCurrentRobotStateMutexName() {
  return current_robot_state_mutex_name_;
}

int SharedMemoryInfo::getParameterMemorySize(int index) {
  if (index == 0) {
    return params_memory_size_0_;
  } else if (index == 1) {
    return params_memory_size_1_;
  } else {
    assert(false);
    return 0;
  }
}

int SharedMemoryInfo::getSensorDataMemorySize() {
  return sensor_buffer_size_;
}

int SharedMemoryInfo::getObjectMemorySize() {
  return objects_memory_size_;
}

int SharedMemoryInfo::getExecutionResponseMemorySize() {
  return execution_response_feedback_size_ + execution_response_result_size_;
}

int SharedMemoryInfo::getCurrentRobotStateMemorySize() {
  return current_robot_state_size_;
}

int SharedMemoryInfo::getSizeForTrajectoryParameters() {
  return trajectory_params_buffer_size_;
}

int SharedMemoryInfo::getOffsetForTrajectoryParameters() {
  return 0;
}


int SharedMemoryInfo::getSizeForFeedbackControllerParameters() {
  return feedback_controller_params_buffer_size_;
}

int SharedMemoryInfo::getOffsetForFeedbackControllerParameters() {
  return trajectory_params_buffer_size_;
}



int SharedMemoryInfo::getSizeForTerminationParameters() {
  return termination_params_buffer_size_;
}

int SharedMemoryInfo::getOffsetForTerminationParameters() {
  return trajectory_params_buffer_size_ + feedback_controller_params_buffer_size_;
}

int SharedMemoryInfo::getSizeForTimerParameters() {
  return timer_params_buffer_size_;
}

int SharedMemoryInfo::getOffsetForTimerParameters() {
  return (trajectory_params_buffer_size_
    + feedback_controller_params_buffer_size_
    + termination_params_buffer_size_);
}

int SharedMemoryInfo::getSizeForSensorData()  {
    return sensor_buffer_size_;
}

int SharedMemoryInfo::getOffsetForSensorDataTrajectoryGenerator()  {
  return (trajectory_params_buffer_size_
          + feedback_controller_params_buffer_size_
          + termination_params_buffer_size_
          + timer_params_buffer_size_);
}

int SharedMemoryInfo::getOffsetForSensorDataFeedbackController()  {
  return (trajectory_params_buffer_size_
          + feedback_controller_params_buffer_size_
          + termination_params_buffer_size_
          + timer_params_buffer_size_
          + sensor_buffer_size_
          );
}

int SharedMemoryInfo::getOffsetForSensorDataTerminationHandler()  {
  return (trajectory_params_buffer_size_
          + feedback_controller_params_buffer_size_
          + termination_params_buffer_size_
          + timer_params_buffer_size_
          + sensor_buffer_size_ * 2
          );
}

int SharedMemoryInfo::getSizeForExecutionFeedbackData() {
  return execution_response_feedback_size_;
}

int SharedMemoryInfo::getOffsetForExecutionFeedbackData() {
  return 0;
}

int SharedMemoryInfo::getSizeForExecutionResultData() {
  return execution_response_result_size_;
}

int SharedMemoryInfo::getOffsetForExecutionResultData() {
  return execution_response_feedback_size_;
}

int SharedMemoryInfo::getSizeForCurrentRobotState() {
  return current_robot_state_size_;
}

int SharedMemoryInfo::getOffsetForCurrentRobotState() {
  return 0;
}