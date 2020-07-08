#ifndef FRANKA_INTERFACE_SENSOR_DATA_MANAGER_H
#define FRANKA_INTERFACE_SENSOR_DATA_MANAGER_H

#include <boost/interprocess/sync/interprocess_mutex.hpp>

#include <franka-interface-common/definitions.h>
#include <franka-interface-common/SharedMemoryInfo.h>

#include "sensor_msg.pb.h"


/**
 * This class will try to read the sensor data from shared memory and populate the appropriate MessageProto. For each
 * class  This class tries to acquire the lock to read the sensor data part of shared memory. If successful, it writes
 * the data into shared memory, else if it cannot acquire the lock it does not do anything.
 *
 * The protocol for reading data to the shared memory is the following. Note that the shared memory is of type
 * unsigned int (uint_8) i.e. raw bytes.
 *
 * 1) First byte of the shared memory is read and should be 1 (indicating a new message). If the first byte is 0 do not
 *    read the message.
 *
 * 2) The second byte is the type of shared memory message type. This should be used to verify if the right
 *    message is being read by the franka-interface library running on control-PC. This is arbitrarily set for now.
 *    More importantly, this is just a single byte for now so the type value should lie between 0 and 255.
 *
 * 3) In the next 4 bytes (i.e. byte 2 to 6) we read the size of the sensor data message that was written. We read
 *    the lowest byte (2) as the least significant byte of the integer size and so on.
 *
 * 4) From byte 6 onwards we use the protobuf library to parse the raw proto data into a Protobuf message.
 */
class SensorDataManager {
 public:
  SensorDataManager(SensorBufferTypePtr trajectory_generator_buffer,
                    SensorBufferTypePtr feedback_controller_buffer,
                    SensorBufferTypePtr termination_handler_buffer,
                    boost::interprocess::interprocess_mutex *mutex) :
                      trajectory_generator_buffer_(trajectory_generator_buffer),
                      feedback_controller_buffer_(feedback_controller_buffer),
                      termination_handler_buffer_(termination_handler_buffer),
                      buffer_group_mutex_(mutex)
                      {};
    
    SensorDataManagerReadStatus readTrajectoryGeneratorSensorMessage(google::protobuf::Message& message);
    SensorDataManagerReadStatus readFeedbackControllerSensorMessage(google::protobuf::Message& message);
    SensorDataManagerReadStatus readTerminationHandlerSensorMessage(google::protobuf::Message& message);

    /**
     * Clears buffer
     */
    void clearBuffers();

    boost::interprocess::interprocess_mutex* getSensorBufferGroupMutex();

 private:
    SensorBufferTypePtr trajectory_generator_buffer_;
    SensorBufferTypePtr feedback_controller_buffer_;
    SensorBufferTypePtr termination_handler_buffer_;
    boost::interprocess::interprocess_mutex* buffer_group_mutex_ = nullptr;

    SensorDataManagerReadStatus readSensorMessage(google::protobuf::Message& message, SensorBufferTypePtr buffer);
    SensorDataManagerReadStatus readMessageAsBytes(std::function< bool(const void *bytes, int data_size)> parse_callback, SensorBufferTypePtr buffer);

    SharedMemoryInfo shared_memory_info_ = SharedMemoryInfo();
};

#endif //FRANKA_INTERFACE_SENSOR_DATA_MANAGER_H
