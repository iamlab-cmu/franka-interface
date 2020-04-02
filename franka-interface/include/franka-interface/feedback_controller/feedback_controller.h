#ifndef FRANKA_INTERFACE_FEEDBACK_CONTROLLER_FEEDBACK_CONTROLLER_H_
#define FRANKA_INTERFACE_FEEDBACK_CONTROLLER_FEEDBACK_CONTROLLER_H_

#include <iostream>
#include <google/protobuf/message.h>
#include <array>
#include <franka/robot_state.h>
#include <franka-interface-common/definitions.h>

#include "franka-interface/trajectory_generator/trajectory_generator.h"
#include "franka-interface/franka_robot.h"
#include "feedback_controller_params_msg.pb.h"

class FeedbackController {
 public:
  explicit FeedbackController(SharedBufferTypePtr p, SensorDataManager* sensor_data_manager) : 
                                                                                params_{p},
                                                                                sensor_data_manager_{sensor_data_manager} {};

  /**
   * Parse parameters from memory.
   */
  virtual void parse_parameters() = 0;

  /**
   * Initialize trajectory generation after parameter parsing.
   */
  virtual void initialize_controller(FrankaRobot *robot) = 0;

  /**
   *  Get next trajectory step.
   */
  virtual void get_next_step(const franka::RobotState &robot_state, 
                             TrajectoryGenerator *traj_generator) = 0;

  /**
   * Parse sensor data
   */
  virtual void parse_sensor_data(const franka::RobotState &robot_state) {};

  std::array<double, 7> tau_d_array_{};

 protected:
  SharedBufferTypePtr params_=0;
  SensorDataManager* sensor_data_manager_;
};

#endif  // FRANKA_INTERFACE_FEEDBACK_CONTROLLER_FEEDBACK_CONTROLLER_H_