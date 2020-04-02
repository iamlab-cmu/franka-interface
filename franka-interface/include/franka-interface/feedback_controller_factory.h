#ifndef FRANKA_INTERFACE_FEEDBACK_CONTROLLER_FACTORY_H_
#define FRANKA_INTERFACE_FEEDBACK_CONTROLLER_FACTORY_H_

#include "franka-interface-common/definitions.h"
#include "franka-interface/sensor_data_manager.h"

class FeedbackController;

class FeedbackControllerFactory {
 public:
  FeedbackControllerFactory() {};

  /**
   * Get feedback controller for skill.
   *
   * @param memory_region  Region of the memory where the parameters
   * will be stored.
   * @return FeedbackController instance for this skill
   */
  FeedbackController* getFeedbackControllerForSkill(SharedBufferTypePtr buffer, SensorDataManager* sensor_data_manager);

};

#endif  // FRANKA_INTERFACE_FEEDBACK_CONTROLLER_FACTORY_H_