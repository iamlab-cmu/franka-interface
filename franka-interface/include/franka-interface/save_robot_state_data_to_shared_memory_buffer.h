#ifndef FRANKA_INTERFACE_SAVE_ROBOT_STATE_TO_SHARED_MEMORY_BUFFER_H_
#define FRANKA_INTERFACE_SAVE_ROBOT_STATE_TO_SHARED_MEMORY_BUFFER_H_

#include <cstring>
#include <iostream>
#include <google/protobuf/message.h>

#include "franka-interface/robot_state_data.h"
#include "franka-interface/run_loop_shared_memory_handler.h"
#include "robot_state_msg.pb.h"

#include <franka-interface-common/definitions.h>

void save_current_robot_state_data_to_shared_memory_buffer(RunLoopSharedMemoryHandler* shared_memory_handler,
                                                           RobotStateData* robot_state_data);

#endif  // FRANKA_INTERFACE_SAVE_ROBOT_STATE_TO_SHARED_MEMORY_BUFFER_H_