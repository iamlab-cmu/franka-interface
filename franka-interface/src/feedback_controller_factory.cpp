//
// Created by mohit on 12/18/18.
//

#include "franka-interface/feedback_controller_factory.h"

#include <iostream>

#include <franka-interface-common/definitions.h>

#include "franka-interface/feedback_controller/cartesian_impedance_feedback_controller.h"
#include "franka-interface/feedback_controller/force_axis_impedence_feedback_controller.h"
#include "franka-interface/feedback_controller/force_position_feedback_controller.h"
#include "franka-interface/feedback_controller/joint_impedance_feedback_controller.h"
#include "franka-interface/feedback_controller/noop_feedback_controller.h"
#include "franka-interface/feedback_controller/pass_through_feedback_controller.h"
#include "franka-interface/feedback_controller/set_internal_impedance_feedback_controller.h"

FeedbackController* FeedbackControllerFactory::getFeedbackControllerForSkill(SharedBufferTypePtr buffer, SensorDataManager* sensor_data_manager){
  FeedbackControllerType feedback_controller_type = static_cast<FeedbackControllerType>(buffer[0]);

  std::cout << "Feedback Controller Type: " << 
  static_cast<std::underlying_type<FeedbackControllerType>::type>(feedback_controller_type) << 
  "\n";

  FeedbackController* feedback_controller = nullptr;
  switch (feedback_controller_type) {
    case FeedbackControllerType::NoopFeedbackController:
      feedback_controller = new NoopFeedbackController(buffer, sensor_data_manager);
      break;
    case FeedbackControllerType::CartesianImpedanceFeedbackController:
      feedback_controller = new CartesianImpedanceFeedbackController(buffer, sensor_data_manager);
      break;
    case FeedbackControllerType::JointImpedanceFeedbackController:
      feedback_controller = new JointImpedanceFeedbackController(buffer, sensor_data_manager);
      break;
    case FeedbackControllerType::ForceAxisImpedenceFeedbackController:
      feedback_controller = new ForceAxisImpedenceFeedbackController(buffer, sensor_data_manager);
      break;
    case FeedbackControllerType::ForcePositionFeedbackController:
      feedback_controller = new ForcePositionFeedbackController(buffer, sensor_data_manager);
      break;
    case FeedbackControllerType::PassThroughFeedbackController:
      feedback_controller = new PassThroughFeedbackController(buffer, sensor_data_manager);
      break;
    case FeedbackControllerType::SetInternalImpedanceFeedbackController:
      feedback_controller = new SetInternalImpedanceFeedbackController(buffer, sensor_data_manager);
      break;
    default:
      std::cout << "Cannot create Feedback Controller with type: " << 
      static_cast<std::underlying_type<FeedbackControllerType>::type>(feedback_controller_type) << 
      "\n";
      return nullptr;
  }

  feedback_controller->parse_parameters();

  return feedback_controller;
}
