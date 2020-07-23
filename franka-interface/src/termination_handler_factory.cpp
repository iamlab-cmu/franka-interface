//
// Created by mohit on 12/18/18.
//

#include "franka-interface/termination_handler_factory.h"

#include <iostream>
#include <franka-interface-common/definitions.h>

#include "franka-interface/termination_handler/contact_termination_handler.h"
#include "franka-interface/termination_handler/final_joint_termination_handler.h"
#include "franka-interface/termination_handler/final_pose_termination_handler.h"
#include "franka-interface/termination_handler/noop_termination_handler.h"
#include "franka-interface/termination_handler/time_termination_handler.h"

TerminationHandler* TerminationHandlerFactory::getTerminationHandlerForSkill(SharedBufferTypePtr buffer, RunLoopProcessInfo *run_loop_info, SensorDataManager* sensor_data_manager) {
  TerminationHandlerType termination_handler_type = static_cast<TerminationHandlerType>(buffer[0]);

  std::string termination_handler_type_name;

  TerminationHandler *termination_handler = nullptr;
  switch (termination_handler_type) {
    case TerminationHandlerType::ContactTerminationHandler:
      termination_handler_type_name = "ContactTerminationHandler";
      termination_handler = new ContactTerminationHandler(buffer, run_loop_info, sensor_data_manager);
      break;
    case TerminationHandlerType::FinalJointTerminationHandler:
      termination_handler_type_name = "FinalJointTerminationHandler";
      termination_handler = new FinalJointTerminationHandler(buffer, run_loop_info, sensor_data_manager);
      break;
    case TerminationHandlerType::FinalPoseTerminationHandler:
      termination_handler_type_name = "FinalPoseTerminationHandler";
      termination_handler = new FinalPoseTerminationHandler(buffer, run_loop_info, sensor_data_manager);
      break;
    case TerminationHandlerType::NoopTerminationHandler:
      termination_handler_type_name = "NoopTerminationHandler";
      termination_handler = new NoopTerminationHandler(buffer, run_loop_info, sensor_data_manager);
      break;
    case TerminationHandlerType::TimeTerminationHandler:
      termination_handler_type_name = "TimeTerminationHandler";
      termination_handler = new TimeTerminationHandler(buffer, run_loop_info, sensor_data_manager);
      break;
    default:
      std::cout << "Cannot create Termination Handler with type: " << 
      static_cast<std::underlying_type<TerminationHandlerType>::type>(termination_handler_type) <<
      "\n" ;
      return nullptr;
  }
  std::cout << "Termination Handler Type: " << termination_handler_type_name << std::endl;

  termination_handler->parse_parameters();
  return termination_handler;
}
