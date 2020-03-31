#ifndef FRANKA_INTERFACE_TERMINATION_HANDLER_FACTORY_H_
#define FRANKA_INTERFACE_TERMINATION_HANDLER_FACTORY_H_

#include <franka-interface-common/definitions.h>
#include <franka-interface-common/run_loop_process_info.h>
#include "franka-interface/sensor_data_manager.h"

class TerminationHandler;

class TerminationHandlerFactory {
 public:
  TerminationHandlerFactory() {};

  /**
   * Get termination handler for skill.
   *
   * @param memory_region  Region of the memory where the parameters
   * will be stored.
   * @return TermatinationHanndler instance for this skill
   */
  TerminationHandler* getTerminationHandlerForSkill(SharedBufferTypePtr buffer, 
  													RunLoopProcessInfo *run_loop_info,
                                                    SensorDataManager* sensor_data_manager);

};

#endif  // FRANKA_INTERFACE_TERMINATION_HANDLER_FACTORY_H_