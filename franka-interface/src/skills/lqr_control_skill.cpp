//
// Created by saumya on 10/6/20.
//

#include "franka-interface/skills/lqr_control_skill.h"

#include <cassert>

#include "franka-interface/robot_state_data.h"
#include "franka-interface/run_loop.h"
#include "franka-interface/run_loop_shared_memory_handler.h"

#include <franka-interface-common/definitions.h>
#include <franka-interface-common/run_loop_process_info.h>

void LqrControlSkill::execute_skill_on_franka(run_loop* run_loop, 
                                                    FrankaRobot* robot,
                                                    RobotStateData *robot_state_data) {

  double time = 0.0;
  int log_counter = 0;
  std::array<double, 16> pose_desired;

  RunLoopSharedMemoryHandler* shared_memory_handler = run_loop->get_shared_memory_handler();
  RunLoopProcessInfo* run_loop_info = shared_memory_handler->getRunLoopProcessInfo();
  boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(
                                  *(shared_memory_handler->getRunLoopProcessInfoMutex()),
                                  boost::interprocess::defer_lock);
  SensorDataManager* sensor_data_manager = run_loop->get_sensor_data_manager();

  std::cout << "Will run the control loop\n";
  
  // define callback for the torque control loop
  std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
      lqr_control_callback = [&](const franka::RobotState& robot_state,
                                              franka::Duration period) -> franka::Torques {

    current_period_ = period.toSec();
    time += current_period_;

    // std::cout << "current_period_: " << current_period_ << "\n";

    if (time == 0.0) {
      traj_generator_->initialize_trajectory(robot_state, SkillType::LqrControlSkill);
      traj_generator_->initialize_model(robot);
      try {
        if (lock.try_lock()) {
          run_loop_info->set_time_skill_started_in_robot_time(robot_state.time.toSec());
          lock.unlock();
        } 
      } catch (boost::interprocess::lock_exception) {
        // Do nothing
      }
    }

    if (robot_state_data->mutex_.try_lock()) {
      robot_state_data->counter_ += 1;
      robot_state_data->time_ =  period.toSec();
      robot_state_data->has_data_ = true;
      robot_state_data->mutex_.unlock();
    }

    traj_generator_->time_ = time;
    traj_generator_->dt_ = current_period_;
    traj_generator_->lqrt_ += current_period_;
    feedback_controller_->time_ = time;
    feedback_controller_->dt_ = current_period_;
    // time += period.toSec();
    log_counter += 1;
    try {
      sensor_data_manager->getSensorBufferGroupMutex()->try_lock();
      traj_generator_->parse_sensor_data(robot_state);
      feedback_controller_->parse_sensor_data(robot_state);
      termination_handler_->parse_sensor_data(robot_state);
      sensor_data_manager->getSensorBufferGroupMutex()->unlock();
    } catch (boost::interprocess::lock_exception) {
    }

    if (time > 0.0) {
      traj_generator_->get_next_step(robot_state);
      feedback_controller_->get_next_step(robot_state, traj_generator_);
    }

    if (log_counter % 1 == 0) {
      pose_desired[0] = feedback_controller_->f_task_[0];
      pose_desired[1] = feedback_controller_->f_task_[1];
      pose_desired[2] = feedback_controller_->f_task_[2];
      pose_desired[3] = feedback_controller_->f_task_[3];
      pose_desired[4] = feedback_controller_->f_task_[4];
      pose_desired[5] = feedback_controller_->f_task_[5];

      pose_desired[6] = traj_generator_->object_position_[0];
      pose_desired[7] = traj_generator_->object_position_[1];
      pose_desired[8] = traj_generator_->object_position_[2];
      pose_desired[9] = traj_generator_->object_position_[3];
      pose_desired[10] = traj_generator_->object_position_[4];
      pose_desired[11] = traj_generator_->object_position_[5];
      pose_desired[12] = traj_generator_->object_position_[6];

      robot_state_data->log_robot_state(pose_desired, robot_state, robot->getModel(), time);
    }


    bool done = termination_handler_->should_terminate(robot_state, model_, traj_generator_);

    if (done && time > 0.0) {
      try {
        if (lock.try_lock()) {
          run_loop_info->set_time_skill_finished_in_robot_time(robot_state.time.toSec());
          lock.unlock();
        } 
      } catch (boost::interprocess::lock_exception) {
        // Do nothing
      }
      
      return franka::MotionFinished(franka::Torques(feedback_controller_->tau_d_array_));
    }

    return feedback_controller_->tau_d_array_;
  };

  robot->robot_.control(lqr_control_callback);
}