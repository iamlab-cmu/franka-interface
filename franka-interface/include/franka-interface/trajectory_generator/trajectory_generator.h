#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_TRAJECTORY_GENERATOR_H_

#include <iostream>
#include <google/protobuf/message.h>
#include <franka/robot_state.h>
#include <franka-interface-common/definitions.h>

#include "franka-interface/sensor_data_manager.h"
#include "trajectory_generator_params_msg.pb.h"

class TrajectoryGenerator {
 public:
  explicit TrajectoryGenerator(SharedBufferTypePtr p, SensorDataManager* sensor_data_manager) : 
    params_{p},
    sensor_data_manager_{sensor_data_manager}
    {};

  /**
   * Parse parameters from memory.
   */
  virtual void parse_parameters() = 0;

  /**
   * Initialize trajectory generation after parameter parsing.
   */
  virtual void initialize_trajectory(const franka::RobotState &robot_state,
                                     SkillType skill_type) = 0;

  /**
   *  Get next trajectory step.
   */
  virtual void get_next_step(const franka::RobotState &robot_state) = 0;

  /**
   * Parse sensor data
   */
  virtual void parse_sensor_data(const franka::RobotState &robot_state) {};

  double run_time_ = 0.0;
  double dt_ = 0.001;
  double time_ = 0.0;
  double t_ = 0.0;

 protected:
  SharedBufferTypePtr params_=0;
  SensorDataManager* sensor_data_manager_;

};

#endif  // FRANKA_INTERFACE_TRAJECTORY_GENERATOR_TRAJECTORY_GENERATOR_H_