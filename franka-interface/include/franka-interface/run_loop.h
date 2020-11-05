#ifndef FRANKA_INTERFACE_RUN_LOOP_H_
#define FRANKA_INTERFACE_RUN_LOOP_H_

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <thread>
#include <iostream>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/filesystem.hpp> 

#include <franka-interface-common/definitions.h>
#include <franka-interface-common/run_loop_process_info.h>
#include <franka-interface-common/SharedMemoryInfo.h>

#include "franka-interface/skill_info_manager.h"
#include "franka-interface/run_loop_logger.h"
#include "franka-interface/robot_state_data.h"
#include "franka-interface/run_loop_shared_memory_handler.h"
#include "franka-interface/trajectory_generator_factory.h"
#include "franka-interface/feedback_controller_factory.h"
#include "franka-interface/termination_handler_factory.h"
#include "franka-interface/franka_gripper.h"
#include "franka-interface/franka_robot.h"
#include "franka-interface/sensor_data_manager.h"

// Set thread to real time priority.
void setCurrentThreadToRealtime(bool throw_on_error);

// TODO(Mohit): Add a namespace to these declarations.

class run_loop {
 public:
  run_loop(std::mutex& logger_mutex,
           std::mutex& robot_loop_data_mutex,
           std::string robot_ip,
           bool stop_on_error,
           bool reset_skill_numbering_on_error,
           bool use_new_filestream_on_error,
           bool log,
           std::string logdir,
           bool with_gripper
          )  :  logger_(logger_mutex),
                process_info_requires_update_(false),
                limit_rate_(false),
                cutoff_frequency_(0.0),
                elapsed_time_(0.0),
                stop_on_error_(stop_on_error),
                reset_skill_numbering_on_error_(reset_skill_numbering_on_error),
                use_new_filestream_on_error_(use_new_filestream_on_error),
                log_(log),
                logdir_(logdir),
                with_gripper_(with_gripper)
  {

    robot_state_data_ = new RobotStateData(robot_loop_data_mutex, log_);
    robot_ = new FrankaRobot(robot_ip);
    if (with_gripper_ == 1) {
      gripper_ = new FrankaGripper(robot_ip);
    } else {
      gripper_ = nullptr;
    }
    if (log_) {
      if (logdir_.back() == '/') {
        logdir_.pop_back();
      }
      boost::filesystem::path boost_logdir(logdir_);
      if (boost::filesystem::create_directory(boost_logdir)) {
          std::cout << "Logdir didn't exist, so it was created: " << logdir_ << std::endl;
      }
    }
  };

  // Todo(Mohit): Implement this!!! We should free up the shared memory correctly.
  // ~run_loop();

  void init();

  /**
   *  Start the RunLoop.
   *
   *  This will allocate the shared memory buffers i.e., shared memory object and
   *  shared memory segment used to communicate between the actionlib interface and
   *  the real time loop.
   */
  void start();

  void stop();

  /**
   *  Update the currently executing task. Maybe we should pass in the TaskInfo or
   *  it should return some task info from it.
   */
  bool update();

  /**
   *  Start running the real time loop on franka
   */
  void run_on_franka();

  /**
   * Get SkillInfo manager.
   */
  SkillInfoManager* getSkillInfoManager();

  /**
   * Did finish skill in meta skill.
   * @param skill
   */
  void didFinishSkillInMetaSkill(BaseSkill* skill);

  /**
   * Start executing new skill.
   * @param new_skill New skill to start.
   */
  void start_new_skill(BaseSkill* new_skill);

  /**
   *  Finish current executing skill.
   */
  void finish_current_skill(BaseSkill* skill);

  void write_skill_result_to_shared_memory(BaseSkill* skill);

  RunLoopSharedMemoryHandler* get_shared_memory_handler();

  static std::atomic<bool> run_loop_ok_;

  SensorDataManager* get_sensor_data_manager();
  bool with_gripper_;

 private:

  FrankaRobot* robot_;
  FrankaGripper* gripper_;
  static std::mutex robot_access_mutex_;

  std::thread robot_state_read_thread_{};
  std::thread current_robot_state_io_thread_{};
  std::thread watchdog_thread_{};

  RunLoopSharedMemoryHandler* shared_memory_handler_ = nullptr;
  SkillInfoManager skill_manager_{};
  RunLoopLogger logger_;
  // This logs the robot state data by using robot readState and within control loops.
  RobotStateData* robot_state_data_ = nullptr;

  // If this flag is true at every loop we will try to get the lock and update process info.
  bool process_info_requires_update_;
  const bool limit_rate_;  // NOLINT(readability-identifier-naming)

  const double cutoff_frequency_; // NOLINT(readability-identifier-naming)
  uint32_t elapsed_time_;
  bool stop_on_error_;
  bool reset_skill_numbering_on_error_;
  bool use_new_filestream_on_error_;
  bool log_;

  std::string logdir_;

  TrajectoryGeneratorFactory traj_gen_factory_={};
  FeedbackControllerFactory feedback_controller_factory_={};
  TerminationHandlerFactory termination_handler_factory_={};
  SensorDataManager* sensor_data_manager_ = nullptr;

  void set_franka_interface_status(bool is_ready, std::string error_message);

  /**
   * Check if new skill should be started or not. Starting a new skill
   * initializes it's trajectory generator, feedback controller and other
   * associated things.
   *
   * @param old_skill
   * @param new_skill
   * @return True if new skill should be started else false.
   */
  bool should_start_new_skill(BaseSkill* old_skill, BaseSkill* new_skill);

  /**
   * Update process info in the shared memory to reflect run-loop's
   * current status.
   */
  void update_process_info();

  /**
   * Setup thread to print data from the real time control loop thread.
   */
  void setup_save_robot_state_thread();

  /**
   * Setup thread to save current robot state data to shared memory buffer.
   */
  void setup_current_robot_state_io_thread();

  /**
   * Setup thread to reset watchdog counter.
   */
  void setup_watchdog_thread();

  /**
   * Setup default collision behavior for robot.
   */
  void setup_robot_default_behavior();

  /**
   * Setup data loggers for logging robot state and larger control loop data.
   */
  void setup_data_loggers();

  /**
   * Log skill description to file logger. Only logs once when the skill begins.
   * @param skill Skill to log.
   */
  void log_skill_info(BaseSkill* skill);
};

#endif  // FRANKA_INTERFACE_ROBOT_STATE_DATA_H_