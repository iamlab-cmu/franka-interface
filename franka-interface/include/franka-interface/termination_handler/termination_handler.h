#ifndef FRANKA_INTERFACE_TERMINATION_HANDLER_TERMINATION_HANDLER_H_
#define FRANKA_INTERFACE_TERMINATION_HANDLER_TERMINATION_HANDLER_H_

#include <iostream>
#include <vector>
#include <array>
#include <Eigen/Dense>
#include <franka/robot_state.h>
#include <google/protobuf/message.h>

#include <franka-interface-common/definitions.h>
#include <franka-interface-common/run_loop_process_info.h>
#include "franka-interface/trajectory_generator/trajectory_generator.h"
#include "franka-interface/franka_robot.h"
#include "franka-interface/sensor_data_manager.h"
#include "termination_handler_params_msg.pb.h"

class TerminationHandler {
 public:
  explicit TerminationHandler(SharedBufferTypePtr p, RunLoopProcessInfo *r, SensorDataManager* sensor_data_manager) : 
                                                  params_{p}, 
                                                  run_loop_info_{r},
                                                  sensor_data_manager_{sensor_data_manager}
  {};

  /**
   * Parse parameters from memory.
   */
  virtual void parse_parameters() = 0;

  /**
   * Initialize termination handler after parameter parsing.
   */
  virtual void initialize_handler(FrankaRobot *robot) = 0;

  /**
   * Should we terminate the current skill.
   */
  virtual bool should_terminate(const franka::RobotState &robot_state, 
                                          franka::Model* robot_model,
                                          TrajectoryGenerator *traj_generator) = 0;

  /**
   * Check if skill terminated previously. By default check the done_ flag and 
   * return it.
   * @return  True if skill has terminated previously else False.
   */
  bool has_terminated();

  bool has_terminated_by_virt_coll();

  /**
   * Sets done_ to true if preempt flag is true.
   */
  void check_terminate_preempt();

  void check_terminate_time(TrajectoryGenerator *trajectory_generator);

  void check_terminate_virtual_wall_collisions(const franka::RobotState &robot_state, franka::Model *robot_model);

  /**
   * Parse sensor data
   */
  virtual void parse_sensor_data(const franka::RobotState &robot_state);

  bool done_ = false;

  double buffer_time_{0.0};

 protected:
  SharedBufferTypePtr params_ = 0;
  RunLoopProcessInfo *run_loop_info_ = nullptr;
  SensorDataManager* sensor_data_manager_;
  ShouldTerminateSensorMessage should_terminate_msg_;

  // Create hyperplanes
  const std::vector<Eigen::Hyperplane<double,3>> planes_ {
    // Front
    Eigen::Hyperplane<double, 3>(Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(0.75, 0., 0.)),
    // Sides 
    Eigen::Hyperplane<double, 3>(Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(0., 0.47, 0.)),
    Eigen::Hyperplane<double, 3>(Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(0., -0.47, 0.)),
    // Bottom
    Eigen::Hyperplane<double, 3>(Eigen::Vector3d(0., 0., 1.), Eigen::Vector3d(0., 0., -0.015)),
    // Top
    Eigen::Hyperplane<double, 3>(Eigen::Vector3d(0., 0., 1.), Eigen::Vector3d(0., 0., 1.25)),
    // Back
    Eigen::Hyperplane<double, 3>(Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(-0.46, 0., 0.))
  };

  // Create dist thresholds
  const std::array<double, 7> dist_thresholds_ = std::array<double, 7>{0.11, 0.11, 0.08, 0.08, 0.07, 0.07, 0.1};

  bool terminated_by_virt_coll_ = false;

};

#endif  // FRANKA_INTERFACE_TERMINATION_HANDLER_TERMINATION_HANDLER_H_