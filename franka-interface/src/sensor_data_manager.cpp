#include "franka-interface/sensor_data_manager.h"

#include <iostream>

#include <google/protobuf/message.h>

SensorDataManagerReadStatus SensorDataManager::readTrajectoryGeneratorSensorMessage(google::protobuf::Message &message) {
   return readSensorMessage(message, trajectory_generator_buffer_);
}

SensorDataManagerReadStatus SensorDataManager::readFeedbackControllerSensorMessage(google::protobuf::Message &message) {
   return readSensorMessage(message, feedback_controller_buffer_);
}

SensorDataManagerReadStatus SensorDataManager::readTerminationHandlerSensorMessage(google::protobuf::Message &message) {
   return readSensorMessage(message, termination_handler_buffer_);
}

SensorDataManagerReadStatus SensorDataManager::readSensorMessage(google::protobuf::Message &message, SensorBufferTypePtr buffer) {
   std::function<bool(const void *bytes, int data_size)>
       parse_callback = [&](const void *bytes, int data_size) -> bool {
     // get state variables
     return message.ParseFromArray(bytes, data_size);
   };
   return readMessageAsBytes(parse_callback, buffer);
}

SensorDataManagerReadStatus SensorDataManager::readMessageAsBytes(
  std::function< bool(const void *bytes, int data_size)> parse_callback, SensorBufferTypePtr buffer) {
  SensorDataManagerReadStatus status;

  uint8_t has_new_message = buffer[0];
  if (has_new_message == 1) {
    uint8_t sensor_msg_type = buffer[1]; // Currently not used
    int data_size = (buffer[2] + (buffer[3] << 8) + (buffer[4] << 16) + (buffer[5] << 24));

    if (parse_callback(buffer + 6, data_size)) {
      status = SensorDataManagerReadStatus::SUCCESS;
      buffer[0] = 0;
    } else {
      status = SensorDataManagerReadStatus::FAIL_TO_READ;
    }
  } else {
    status = SensorDataManagerReadStatus::NO_NEW_MESSAGE;
  }

  return status;
}

void SensorDataManager::clearBuffers() {
  try {
    if (buffer_group_mutex_->try_lock()) {
      int sensor_buffer_size = shared_memory_info_.getSizeForSensorData();
      memset(trajectory_generator_buffer_, 0, sensor_buffer_size);
      memset(feedback_controller_buffer_, 0, sensor_buffer_size);
      memset(termination_handler_buffer_, 0, sensor_buffer_size);
      buffer_group_mutex_->unlock();
    }
  } catch (boost::interprocess::lock_exception) {
  }
}

boost::interprocess::interprocess_mutex* SensorDataManager::getSensorBufferGroupMutex() {
  return buffer_group_mutex_;
}