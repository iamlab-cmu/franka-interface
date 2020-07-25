#include "franka-interface/run_loop.h"

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <pthread.h>

#include <cerrno>
#include <cstring>
#include <exception>
#include <fstream>
#include <string>
#include <cassert>

#include <franka/exception.h>
#include <franka-interface-common/definitions.h>

#include "franka-interface/duration.h"
#include "franka-interface/file_stream_logger.h"
#include "franka-interface/robot_state_data.h"
#include "franka-interface/save_robot_state_data_to_shared_memory_buffer.h"
#include "franka-interface/skills/base_meta_skill.h"
#include "franka-interface/skills/base_skill.h"
#include "franka-interface/skills/cartesian_pose_skill.h"
#include "franka-interface/skills/force_torque_skill.h"
#include "franka-interface/skills/gripper_skill.h"
#include "franka-interface/skills/impedance_control_skill.h"
#include "franka-interface/skills/joint_position_continuous_skill.h"
#include "franka-interface/skills/joint_position_skill.h"
#include "franka-interface/utils/logger_utils.h"

std::atomic<bool> run_loop::run_loop_ok_{false};
std::mutex run_loop::robot_access_mutex_;

template<typename ... Args>
std::string string_format(const std::string& format, Args ... args )
{
  size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
  std::unique_ptr<char[]> buf( new char[ size ] );
  snprintf( buf.get(), size, format.c_str(), args ... );
  return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

void setCurrentThreadToRealtime(bool throw_on_error) {
  // Change prints to exceptions.
  const int thread_priority = sched_get_priority_max(SCHED_FIFO);
  if (thread_priority == -1) {
    std::cout << std::string("libfranka: unable to get maximum possible thread priority: ") +
        std::strerror(errno);
  }
  sched_param thread_param{};
  thread_param.sched_priority = thread_priority;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param) != 0) {
    if (throw_on_error) {
      std::cout << std::string("libfranka: unable to set realtime scheduling: ") +
          std::strerror(errno);
    }
  }
}

void run_loop::init() {
  bool throw_on_error = false;
  setCurrentThreadToRealtime(throw_on_error);
  shared_memory_handler_ = new RunLoopSharedMemoryHandler();
}

void run_loop::start() {
  init();
  // Start processing, might want to do some pre-processing 
  std::cout << "start run loop.\n";
  shared_memory_handler_->start();

  sensor_data_manager_ = new SensorDataManager(
      shared_memory_handler_->getSensorDataTrajectoryGeneratorBuffer(0),
      shared_memory_handler_->getSensorDataFeedbackControllerBuffer(0),
      shared_memory_handler_->getSensorDataTerminationHandlerBuffer(0),
      shared_memory_handler_->getSensorDataGroupBufferMutex());
}

void run_loop::stop() {
  // Maybe call this after exceptions or SIGINT or any Interrupt.
  // Stop the interface gracefully.
}

bool run_loop::should_start_new_skill(BaseSkill* old_skill, BaseSkill* new_skill) {
  // No new skill to start.
  if (new_skill == nullptr) {
    return false;
  }
  // Old  skill was null, new skill is not null. should start it.
  if (old_skill == nullptr) {
    return true;
  }
  // If new skill is different than old skill, we should start it.
  if (new_skill->get_skill_id() != old_skill->get_skill_id()) {
    return true;
  }

  return false;
}

void run_loop::start_new_skill(BaseSkill* new_skill) {
  // Generate things that are required here.
  RunLoopProcessInfo* run_loop_info = shared_memory_handler_->getRunLoopProcessInfo();
  int memory_index = run_loop_info->get_current_shared_memory_index();
  std::cout << string_format("Create skill from memory index: %d\n", memory_index);

  SharedBufferTypePtr traj_buffer = shared_memory_handler_->getTrajectoryGeneratorBuffer(memory_index);
  TrajectoryGenerator *traj_generator = traj_gen_factory_.getTrajectoryGeneratorForSkill(
      traj_buffer, sensor_data_manager_);

  SharedBufferTypePtr feedback_controller_buffer = shared_memory_handler_->getFeedbackControllerBuffer(
      memory_index);
  FeedbackController *feedback_controller =
      feedback_controller_factory_.getFeedbackControllerForSkill(feedback_controller_buffer, sensor_data_manager_);

  SharedBufferTypePtr termination_handler_buffer = shared_memory_handler_->getTerminationParametersBuffer(
      memory_index);
  TerminationHandler* termination_handler =
      termination_handler_factory_.getTerminationHandlerForSkill(termination_handler_buffer, run_loop_info, sensor_data_manager_);

  // Start skill, does any pre-processing if required.
  sensor_data_manager_->clearBuffers();
  new_skill->start_skill(robot_, traj_generator, feedback_controller, termination_handler);
}

void run_loop::finish_current_skill(BaseSkill* skill) {
  SkillStatus status = skill->get_current_skill_status();

  if (skill->has_terminated(robot_) && status != SkillStatus::FINISHED) {
    write_skill_result_to_shared_memory(skill);
  }

  status = skill->get_current_skill_status();
  if (status == SkillStatus::FINISHED) {
    process_info_requires_update_ = true;
  }

  if (status == SkillStatus::VIRT_COLL_ERR) {
    throw franka::Exception("Robot is in collision with virtual walls!");
  }

  setup_robot_default_behavior();
  // TODO(Mohit): Do any other-preprocessing if required
}

void run_loop::write_skill_result_to_shared_memory(BaseSkill* skill) {
  SkillStatus status = skill->get_current_skill_status();
  
  if (skill->has_terminated_by_virt_coll()) {
    skill->set_skill_status(SkillStatus::VIRT_COLL_ERR);
  } else {
    skill->set_skill_status(SkillStatus::FINISHED);
  }
  

  // Write results to memory
  int memory_index = skill->get_skill_id() % 2;

  std::cout << "Writing to execution result buffer number: " << memory_index << std::endl;

  SharedBufferTypePtr buffer = shared_memory_handler_->getExecutionResultBuffer(memory_index);
  skill->write_result_to_shared_memory(buffer, robot_);
}

void run_loop::update_process_info() {
  BaseSkill* skill = skill_manager_.get_current_skill();
  
  int current_skill_id = -1;

  if (skill != nullptr) {
    current_skill_id = skill->get_skill_id();
  }
  bool is_executing_skill = skill_manager_.is_currently_executing_skill();

  // Grab the lock and update process info.
  {
    RunLoopProcessInfo* run_loop_info = shared_memory_handler_->getRunLoopProcessInfo();
    boost::interprocess::scoped_lock<
            boost::interprocess::interprocess_mutex> lock(
                *(shared_memory_handler_->getRunLoopProcessInfoMutex()),
                boost::interprocess::defer_lock);
    try {
      if (lock.try_lock()) {
        run_loop_info->set_is_running_skill(is_executing_skill);

        // We have a skill that we have finished. Make sure we update this in RunLoopProcessInfo.
        if (skill != nullptr && !is_executing_skill) {
          if (run_loop_info->get_done_skill_id() > current_skill_id) {
            // Make sure get done skill id is not ahead of us.
            std::cout << string_format("INVALID: RunLoopProcInfo has done skill id %d "
                                                " greater than current skill id %d\n",
                                                run_loop_info->get_done_skill_id(),
                                                current_skill_id);
          } else if (run_loop_info->get_result_skill_id() + 2 <= current_skill_id) {
            // Make sure that ActionLib has read the skill results before we overwrite them.
            std::cout << string_format("ActionLib server has not read previous result %d. "
                              "Cannot write new result %d\n",
                              run_loop_info->get_result_skill_id(),
                              current_skill_id);
          } else if (run_loop_info->get_done_skill_id() != current_skill_id - 1) {
            // Make sure we are only updating skill sequentially.
            // std::cout << string_format("RunLoopProcInfo done skill id: %d current skill id: %d\n",
            //         run_loop_info->get_done_skill_id(), current_skill_id);
          } else {
            run_loop_info->set_done_skill_id(current_skill_id);
            std::cout << string_format("Did set done_skill_id %d\n", current_skill_id);
          }
        }
        process_info_requires_update_ = false;

        // Check if new skill is available only if no current skill is being
        // currently executed.
        if (!is_executing_skill && run_loop_info->get_new_skill_available()) {

          std::cout << "Did get new skill" << std::endl;
          // Create new task Skill
          int new_skill_id = run_loop_info->get_new_skill_id();
          SkillType new_skill_type = static_cast<SkillType>(run_loop_info->get_new_skill_type());
          int new_meta_skill_id = run_loop_info->get_new_meta_skill_id();
          MetaSkillType new_meta_skill_type = static_cast<MetaSkillType>(run_loop_info->get_new_meta_skill_type());
          std::string new_skill_description = run_loop_info->get_new_skill_description();
          std::cout << string_format("Did find new skill id: %d, type: %d meta skill: %d, type: %d\n",
              new_skill_id, new_skill_type, new_meta_skill_id, new_meta_skill_type);

          // Set the current skill to the new skill parameters
          run_loop_info->set_current_skill_id(new_skill_id);
          run_loop_info->set_current_skill_type(static_cast<int>(new_skill_type));
          run_loop_info->set_current_meta_skill_id(new_meta_skill_id);
          run_loop_info->set_current_meta_skill_type(static_cast<int>(new_meta_skill_type));
          run_loop_info->set_current_skill_description(new_skill_description);
          run_loop_info->reset_time_skill_started_in_robot_time();
          run_loop_info->reset_time_skill_finished_in_robot_time();

          BaseSkill *new_skill;

          std::string skill_type_name;
          switch(new_skill_type) {
            case SkillType::CartesianPoseSkill:
              skill_type_name = "CartesianPoseSkill";
              new_skill = new CartesianPoseSkill(new_skill_id, new_meta_skill_id, new_skill_description);
              break;
            case SkillType::ForceTorqueSkill:
              skill_type_name = "ForceTorqueSkill";
              new_skill = new ForceTorqueSkill(new_skill_id, new_meta_skill_id, new_skill_description);
              break;
            case SkillType::GripperSkill:
              skill_type_name = "GripperSkill";
              new_skill = new GripperSkill(new_skill_id, new_meta_skill_id, new_skill_description);
              break;
            case SkillType::ImpedanceControlSkill:
              skill_type_name = "ImpedanceControlSkill";
              new_skill = new ImpedanceControlSkill(new_skill_id, new_meta_skill_id, new_skill_description);
              break;
            case SkillType::JointPositionSkill:
              skill_type_name = "JointPositionSkill";
              new_skill = new JointPositionSkill(new_skill_id, new_meta_skill_id, new_skill_description);
              break;
            default:
              std::cout << "Incorrect skill type: " << 
              static_cast<std::underlying_type<SkillType>::type>(new_skill_type) << 
              "\n";
              assert(false);
          }

          std::cout << "Skill Type: " << skill_type_name << std::endl;

          skill_manager_.add_skill(new_skill);

          // Get Meta-skill
          // BaseMetaSkill* new_meta_skill = skill_manager_.get_meta_skill_with_id(new_meta_skill_id);
          BaseMetaSkill* new_meta_skill = nullptr;
          switch(new_meta_skill_type) {
            case MetaSkillType::BaseMetaSkill:
              new_meta_skill = new BaseMetaSkill(new_meta_skill_id);
              break;
            case MetaSkillType::JointPositionContinuousSkill:
              new_meta_skill = new JointPositionContinuousSkill(new_meta_skill_id);
              break;
            default:
              std::cout << "Incorrect meta skill type: " << 
              static_cast<std::underlying_type<MetaSkillType>::type>(new_meta_skill_type) << 
              "\n";
              assert(false);  
          }
          skill_manager_.add_meta_skill(new_meta_skill);

          // Update the shared memory region. This means that the actionlib service will now write
          // to the other memory region, i.e. not the current memory region.
          // TODO(Mohit): We should lock the other memory so that ActionLibServer cannot modify it?
          run_loop_info->update_shared_memory_region();
          run_loop_info->set_new_skill_available(false);
        } else {          
          // std::cout << "Did not get new skill\n";
        }
      } else {
        std::cout << "Failed to get lock to update process info\n";
      }
    } catch (boost::interprocess::lock_exception) {
      // TODO(Mohit): Do something better here.
      std:: cout << "Cannot acquire lock for run loop info";
    }
  }
}

void run_loop::setup_save_robot_state_thread() {
  int io_rate = 100;   // The below thread will print at 100 FPS.
  robot_state_read_thread_ = std::thread([&, io_rate]() {
      // Sleep to achieve the desired print rate.
      std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

      while (true) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>((1.0 / io_rate * 1000.0))));

        // don't record data if run loop is not ready
        if (!run_loop_ok_) {
          continue;
        }

        // Try to lock data to avoid read write collisions.
        if (robot_access_mutex_.try_lock()) {
          try{
            franka::GripperState gripper_state = robot_->getGripperState();
            franka::RobotState robot_state = robot_->getRobotState();
            
            double duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time).count();
                
            // Make sure update_current_gripper_state is before log_robot_state because log_robot_state will
            // push_back gripper_state info from the current gripper_state
            robot_state_data_->update_current_gripper_state(gripper_state);
            robot_state_data_->log_robot_state(robot_state.O_T_EE_d, robot_state, robot_->getModel(), duration / 1000.0);
          } catch (const franka::Exception& ex) {
            robot_access_mutex_.unlock();
            std::cerr << "Robot state save thread encountered Franka exception. Will not log for now.\n";
          }
          robot_access_mutex_.unlock();
        }
      }
  });
}

void run_loop::setup_current_robot_state_io_thread() {
  int io_rate = 100;
  current_robot_state_io_thread_ = std::thread([&, io_rate]() {
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>((1.0 / io_rate * 1000.0))));

      if (!run_loop_ok_ || robot_state_data_ == nullptr) {
        continue;
      }
      
      save_current_robot_state_data_to_shared_memory_buffer(shared_memory_handler_, robot_state_data_);
    }
  });
}

void run_loop::setup_watchdog_thread() {
  int io_rate = 50;
  watchdog_thread_ = std::thread([&, io_rate]() {
    while (true) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>((1.0 / io_rate * 1000.0))));
        FrankaInterfaceStateInfo* franka_interface_state_info = shared_memory_handler_->getFrankaInterfaceStateInfo();
        boost::interprocess::scoped_lock<
                boost::interprocess::interprocess_mutex> lock(
                    *(shared_memory_handler_->getFrankaInterfaceStateInfoMutex()),
                    boost::interprocess::defer_lock);
        if (lock.try_lock()) {
          franka_interface_state_info->reset_watchdog_counter();
        } 
    }
  });
}

void run_loop::setup_robot_default_behavior() {
  
  // Set additional parameters always before the control loop, NEVER in the control loop!
  // Set collision behavior.
  robot_->robot_.setCollisionBehavior(
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{120.0, 120.0, 118.0, 118.0, 116.0, 114.0, 112.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{120.0, 120.0, 118.0, 118.0, 116.0, 114.0, 112.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{120.0, 120.0, 120.0, 125.0, 125.0, 125.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{120.0, 120.0, 120.0, 125.0, 125.0, 125.0}});
  // TODO(jacky): Use these parameters to make robot super sensitive to collisions. Useful for testing collision error handling.
  // robot_->robot_.setCollisionBehavior(
  //     {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
  //     {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
  //     {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
  //     {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

  robot_->robot_.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot_->robot_.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
  // double m_load = 0.333;
  // std::array<double, 3> F_x_Cload{0.05, 0.05, -0.12};
  // std::array<double, 9> I_load{1, 0, 0, 0, 1, 0, 0, 0, 1};
  // robot_->robot_.setLoad(m_load, F_x_Cload, I_load);
}

void run_loop::didFinishSkillInMetaSkill(BaseSkill* skill) {
  // Finish skill if possible.
  finish_current_skill(skill);
  // Complete old skills and acquire new skills
  update_process_info();
}

void run_loop::setup_data_loggers() {
  // LoggerUtils::all_logger_files();
  int logger_integer_suffix = LoggerUtils::integer_suffix_for_new_log_file(logdir_);
  std::string filename = logdir_ + "/" + "robot_state_data_" + std::to_string(logger_integer_suffix) + ".txt";
  std::cout << "Will save data to: " << filename << std::endl;
  FileStreamLogger *robot_logger = new FileStreamLogger(filename);
  robot_state_data_->setFileStreamLogger(robot_logger);
  robot_state_data_->startFileLoggerThread();
}

void run_loop::log_skill_info(BaseSkill* skill) {
  std::chrono::milliseconds start_time_skill = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch());
  std::string log_desc = string_format("Will execute skill: %d, meta_skill: %d, curr_time: %lld, ",
                                       skill->get_skill_id(), skill->get_meta_skill_id(),
                                       start_time_skill);
  // As convention ALWAYS keep desc: <skill> part in the end. We use parsers to parse skills automatically using
  // this convention.
  log_desc += ("desc: " + skill->get_description());
  robot_state_data_->log_skill_info(log_desc);
  std::cout << log_desc << "\n" << std::endl;
};

void run_loop::set_franka_interface_status(bool is_ready, std::string error_message) {
  FrankaInterfaceStateInfo* franka_interface_state_info = shared_memory_handler_->getFrankaInterfaceStateInfo();
    boost::interprocess::scoped_lock<
            boost::interprocess::interprocess_mutex> lock(
                *(shared_memory_handler_->getFrankaInterfaceStateInfoMutex()),
                boost::interprocess::defer_lock);
  try {
    std::cout << "Will try to acquire lock while setting franka_interface status\n";
    if (lock.try_lock()) {
      franka_interface_state_info->set_is_ready(is_ready);
      franka_interface_state_info->set_error_description(error_message);
    }
  } catch (boost::interprocess::lock_exception) {
    // TODO(Mohit): Do something better here.
    std::cout << "Cannot acquire lock while setting franka_interface status\n";
  }
}

void run_loop::run_on_franka() {
  // Wait for sometime to let the client add data to the buffer
  std::this_thread::sleep_for(std::chrono::seconds(2));

  std::chrono::time_point<std::chrono::high_resolution_clock> start;
  auto milli = std::chrono::milliseconds(1);

  setup_robot_default_behavior();
  setup_watchdog_thread();
  setup_data_loggers();
  setup_current_robot_state_io_thread();
  setup_save_robot_state_thread();

  while (true) {
    BaseSkill* skill;
    try {
      run_loop_ok_ = true;
      set_franka_interface_status(true, "");


      while (true) {
        start = std::chrono::high_resolution_clock::now();

        // Execute the current skill (traj_generator, FBC are here)
        skill = skill_manager_.get_current_skill();
        BaseMetaSkill *meta_skill = skill_manager_.get_current_meta_skill();

        // NOTE: We keep on running the last skill even if it is finished!!
        if (skill != nullptr && meta_skill != nullptr) {
          robot_access_mutex_.lock();
          if (!meta_skill->isComposableSkill() && !skill->get_termination_handler()->done_) {
            // Execute skill.
            log_skill_info(skill);
            meta_skill->execute_skill_on_franka(this, robot_, robot_state_data_);
          } else if (meta_skill->isComposableSkill()) {
            log_skill_info(skill);
            meta_skill->execute_skill_on_franka(this, robot_, robot_state_data_);
          } else {
            finish_current_skill(skill);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
          robot_access_mutex_.unlock();
        }

        // Complete old skills and acquire new skills
        update_process_info();

        // Start new skill, if possible
        BaseSkill* new_skill = skill_manager_.get_current_skill();
        if (should_start_new_skill(skill, new_skill)) {
          std::cout << "Will start skill\n";
          robot_access_mutex_.lock();
          start_new_skill(new_skill);
          robot_access_mutex_.unlock();
        }

        // Sleep to maintain 1Khz frequency, not sure if this is required or not.
        auto finish = std::chrono::high_resolution_clock::now();
        // Wait for start + milli - finish
        auto elapsed = start + milli - finish;
        // TODO(Mohit): We need to sleep for now because the ROS client side sends messasges sequentially
        // and hence we have skills being repeated because the new skill arrives with a delay.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    } catch (const franka::Exception& ex) {
      std::cout << "Caught Franka Exception\n";
      run_loop_ok_ = false;

      // Alert franka_ros_interface that an exception has occurred      
      std::string error_description = ex.what();
      set_franka_interface_status(false, error_description);
      std::cerr << error_description << std::endl;

      // Uncommend below to print the last 50 timesteps for debugging.
      //robot_state_data_->printData(50);

      // Log data
      robot_state_data_->writeCurrentBufferData();

      // Clear buffers and reset stateful variables about skills
      RunLoopProcessInfo* run_loop_info = shared_memory_handler_->getRunLoopProcessInfo();
      boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> run_loop_info_lock(
          *(shared_memory_handler_->getRunLoopProcessInfoMutex())
        );

      if(reset_skill_numbering_on_error_ == 1) {
        std::cout << "Resetting skill variables. \n";
        run_loop_info->reset_skill_vars();
      } else {
        // Set done_skill_id in run loop info
        std::cout << "Setting done skill id to " << skill->get_skill_id() << ".\n";
        write_skill_result_to_shared_memory(skill);
        run_loop_info->set_skill_done_when_error_occurs(skill->get_skill_id());
      }

      skill_manager_.clear_skill_and_meta_skill_list();
      shared_memory_handler_->clearAllBuffers();
      robot_state_data_->clearAllBuffers();

      if(use_new_filestream_on_error_ == 1) {
        // Write new logs to a new log file.
        int logger_integer_suffix = LoggerUtils::integer_suffix_for_new_log_file(logdir_);
        std::string filename = logdir_ + "/" + "robot_state_data_" + std::to_string(logger_integer_suffix) + ".txt";
        std::cout << "Will save data to: " << filename << std::endl;
        robot_state_data_->updateFileStreamLogger(filename);
      }

      // Perform automatic error recovery
      std::cout << "Performing automatic error recovery\n";
      robot_->automaticErrorRecovery();
      robot_access_mutex_.unlock();

      setup_robot_default_behavior();

      std::cout << "Error recovery finished\n";

      // Giving franka_ros_interface some time to react
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      if (stop_on_error_ == 1) {
        // Stop franka_interface immediately on error. This is important when there are unforeseen errors
        // during continuous data collection, which might corrupt the data and recording such events
        // might be hard. By default, we do not stop on error.
        std::cout << "Flag stop_on_error enabled. Will stop franka_interface." << std::endl;
        break;
      } else {
        continue;
      }
    }
  }

  if (robot_state_read_thread_.joinable()) {
    robot_state_read_thread_.join();
  }
  if (current_robot_state_io_thread_.joinable()) {
    current_robot_state_io_thread_.join();
  }
  if (robot_state_data_->file_logger_thread_.joinable()) {
    robot_state_data_->file_logger_thread_.join();
  }
}

SkillInfoManager* run_loop::getSkillInfoManager() {
  return &skill_manager_;
}

RunLoopSharedMemoryHandler* run_loop::get_shared_memory_handler() {
  return shared_memory_handler_;
}

SensorDataManager* run_loop::get_sensor_data_manager() {
  return sensor_data_manager_;
}