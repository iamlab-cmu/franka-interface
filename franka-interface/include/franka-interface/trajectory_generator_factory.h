#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_FACTORY_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_FACTORY_H_

#include "franka-interface-common/definitions.h"
#include "franka-interface/run_loop_shared_memory_handler.h"
#include "franka-interface/sensor_data_manager.h"

class TrajectoryGenerator;

class TrajectoryGeneratorFactory {
 public:

  TrajectoryGeneratorFactory() {};

  /**
   * Get trajectory generator for skill.
   *
   * @param memory_region  Region of the memory where the parameters
   * will be stored.
   */
  TrajectoryGenerator* getTrajectoryGeneratorForSkill(SharedBufferTypePtr buffer, SensorDataManager* sensor_data_manager);

};

#endif  // FRANKA_INTERFACE_TRAJECTORY_GENERATOR_FACTORY_H_