#ifndef FRANKA_INTERFACE_FEEDBACK_CONTROLLER_FACTORY_H_
#define FRANKA_INTERFACE_FEEDBACK_CONTROLLER_FACTORY_H_

#include "franka-interface-common/definitions.h"

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
  FeedbackController* getFeedbackControllerForSkill(SharedBufferTypePtr buffer);

};

#endif  // FRANKA_INTERFACE_FEEDBACK_CONTROLLER_FACTORY_H_