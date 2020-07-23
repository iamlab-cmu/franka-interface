//
// Created by mohit on 12/18/18.
//

#include "franka-interface/feedback_controller_factory.h"

#include <iostream>

#include <franka-interface-common/definitions.h>

#include "franka-interface/feedback_controller/cartesian_impedance_feedback_controller.h"
#include "franka-interface/feedback_controller/ee_cartesian_impedance_feedback_controller.h"
#include "franka-interface/feedback_controller/force_axis_impedence_feedback_controller.h"
#include "franka-interface/feedback_controller/force_position_feedback_controller.h"
#include "franka-interface/feedback_controller/joint_impedance_feedback_controller.h"
#include "franka-interface/feedback_controller/noop_feedback_controller.h"
#include "franka-interface/feedback_controller/pass_through_feedback_controller.h"
#include "franka-interface/feedback_controller/set_internal_impedance_feedback_controller.h"

FeedbackController* FeedbackControllerFactory::getFeedbackControllerForSkill(SharedBufferTypePtr buffer, SensorDataManager* sensor_data_manager){
  FeedbackControllerType feedback_controller_type = static_cast<FeedbackControllerType>(buffer[0]);

  std::string feedback_controller_type_name;

  FeedbackController* feedback_controller = nullptr;
  switch (feedback_controller_type) {
    
    case FeedbackControllerType::CartesianImpedanceFeedbackController:
      feedback_controller_type_name = "CartesianImpedanceFeedbackController";
      feedback_controller = new CartesianImpedanceFeedbackController(buffer, sensor_data_manager);
      break;
    case FeedbackControllerType::EECartesianImpedanceFeedbackController:
      feedback_controller_type_name = "EECartesianImpedanceFeedbackController";
      feedback_controller = new EECartesianImpedanceFeedbackController(buffer, sensor_data_manager);
      break;
    case FeedbackControllerType::ForceAxisImpedenceFeedbackController:
      feedback_controller_type_name = "ForceAxisImpedenceFeedbackController";
      feedback_controller = new ForceAxisImpedenceFeedbackController(buffer, sensor_data_manager);
      break;
    case FeedbackControllerType::ForcePositionFeedbackController:
      feedback_controller_type_name = "ForcePositionFeedbackController";
      feedback_controller = new ForcePositionFeedbackController(buffer, sensor_data_manager);
      break;
    case FeedbackControllerType::JointImpedanceFeedbackController:
      feedback_controller_type_name = "JointImpedanceFeedbackController";
      feedback_controller = new JointImpedanceFeedbackController(buffer, sensor_data_manager);
      break;
    case FeedbackControllerType::NoopFeedbackController:
      feedback_controller_type_name = "NoopFeedbackController";
      feedback_controller = new NoopFeedbackController(buffer, sensor_data_manager);
      break;
    case FeedbackControllerType::PassThroughFeedbackController:
      feedback_controller_type_name = "PassThroughFeedbackController";
      feedback_controller = new PassThroughFeedbackController(buffer, sensor_data_manager);
      break;
    case FeedbackControllerType::SetInternalImpedanceFeedbackController:
      feedback_controller_type_name = "SetInternalImpedanceFeedbackController";
      feedback_controller = new SetInternalImpedanceFeedbackController(buffer, sensor_data_manager);
      break;
    default:
      std::cout << "Cannot create Feedback Controller with type: " << 
      static_cast<std::underlying_type<FeedbackControllerType>::type>(feedback_controller_type) << 
      "\n";
      return nullptr;
  }

  std::cout << "Feedback Controller Type: " << feedback_controller_type_name << std::endl;

  feedback_controller->parse_parameters();

  return feedback_controller;
}
