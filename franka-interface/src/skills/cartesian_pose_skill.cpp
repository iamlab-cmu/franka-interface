#include "franka-interface/skills/cartesian_pose_skill.h"

#include <cassert>
#include <array>

#include <franka/robot.h>

#include "franka-interface/robot_state_data.h"
#include "franka-interface/run_loop.h"
#include "franka-interface/run_loop_shared_memory_handler.h"
#include "franka-interface/feedback_controller/set_internal_impedance_feedback_controller.h"
#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"

#include <franka-interface-common/definitions.h>
#include <franka-interface-common/run_loop_process_info.h>

std::array<double, 16> CartesianPoseSkill::limit_position(std::array<double, 16> &desired_pose, double period) {

  std::array<double, 16> limited_desired_pose = desired_pose;

  for(int i = 0; i < 3; i++) {
    next_position_[i] = desired_pose[12+i];
    next_velocity_[i] = (next_position_[i] - current_position_[i]) / period;

    if(std::abs(next_velocity_[i]) > max_cartesian_translation_velocity_) {
      if(next_velocity_[i] > 0) {
        next_velocity_[i] = max_cartesian_translation_velocity_ * safety_factor;
      } else {
        next_velocity_[i] = -max_cartesian_translation_velocity_ * safety_factor;
      }
    }

    next_acceleration_[i] = (next_velocity_[i] - current_velocity_[i]) / period;

    if(std::abs(next_acceleration_[i]) > max_cartesian_translation_acceleration_) {
      if(next_acceleration_[i] > 0) {
        next_acceleration_[i] = max_cartesian_translation_acceleration_ * safety_factor;
      } else {
        next_acceleration_[i] = -max_cartesian_translation_acceleration_ * safety_factor;
      }
    }

    next_jerk_[i] = (next_acceleration_[i] - current_acceleration_[i]) / period;

    if(std::abs(next_jerk_[i]) > max_cartesian_translation_jerk_) {
      if(next_jerk_[i] > 0) {
        next_jerk_[i] = max_cartesian_translation_jerk_ * safety_factor;
      } else {
        next_jerk_[i] = -max_cartesian_translation_jerk_ * safety_factor;
      }
    }

    next_acceleration_[i] = current_acceleration_[i] + next_jerk_[i] * period;

    if(std::abs(next_acceleration_[i]) > max_cartesian_translation_acceleration_) {
      if(next_acceleration_[i] > 0) {
        next_acceleration_[i] = max_cartesian_translation_acceleration_ * safety_factor;
      } else {
        next_acceleration_[i] = -max_cartesian_translation_acceleration_ * safety_factor;
      }
    }

    next_velocity_[i] = current_velocity_[i] + next_acceleration_[i] * period;

    if(std::abs(next_velocity_[i]) > max_cartesian_translation_velocity_) {
      if(next_velocity_[i] > 0) {
        next_velocity_[i] = max_cartesian_translation_velocity_ * safety_factor;
      } else {
        next_velocity_[i] = -max_cartesian_translation_velocity_ * safety_factor;
      }
    }

    next_position_[i] = current_position_[i] + next_velocity_[i] * period;
    limited_desired_pose[12+i] = next_position_[i];
  }

  return limited_desired_pose;
}

std::array<double, 16> CartesianPoseSkill::limit_position_to_stop(std::array<double, 16> &current_pose, double period) {

  std::array<double, 16> limited_desired_pose = current_pose;

  for(int i = 0; i < 3; i++) {
    if(std::abs(current_velocity_[i]) < eps_) {
      continue;
    } else {
      current_error_[i] = -current_velocity_[i];
      integral_[i] += current_error_[i] * period; 
      derivative_[i] = (current_error_[i] - previous_error_[i]) / period;

      next_acceleration_[i] = Kp_ * current_error_[i] + Ki_ * integral_[i] + Kd_ * derivative_[i];

      previous_error_ = current_error_;

      if(std::abs(next_acceleration_[i]) > max_cartesian_translation_acceleration_) {
        if(next_acceleration_[i] > 0) {
          next_acceleration_[i] = max_cartesian_translation_acceleration_ * safety_factor;
        } else {
          next_acceleration_[i] = -max_cartesian_translation_acceleration_ * safety_factor;
        }
      }

      next_jerk_[i] = (next_acceleration_[i] - current_acceleration_[i]) / period;

      if(std::abs(next_jerk_[i]) > max_cartesian_translation_jerk_) {
        if(next_jerk_[i] > 0) {
          next_jerk_[i] = max_cartesian_translation_jerk_ * safety_factor;
        } else {
          next_jerk_[i] = -max_cartesian_translation_jerk_ * safety_factor;
        }
      }

      next_acceleration_[i] = current_acceleration_[i] + next_jerk_[i] * period;

      if(std::abs(next_acceleration_[i]) > max_cartesian_translation_acceleration_) {
        if(next_acceleration_[i] > 0) {
          next_acceleration_[i] = max_cartesian_translation_acceleration_ * safety_factor;
        } else {
          next_acceleration_[i] = -max_cartesian_translation_acceleration_ * safety_factor;
        }
      }

      next_velocity_[i] = current_velocity_[i] + next_acceleration_[i] * period;

      if(std::abs(next_velocity_[i]) > max_cartesian_translation_velocity_) {
        if(next_velocity_[i] > 0) {
          next_velocity_[i] = max_cartesian_translation_velocity_ * safety_factor;
        } else {
          next_velocity_[i] = -max_cartesian_translation_velocity_ * safety_factor;
        }
      }

      next_position_[i] = current_position_[i] + next_velocity_[i] * period;
      limited_desired_pose[12+i] = next_position_[i];
    }
    
  }

  return limited_desired_pose;
}

void CartesianPoseSkill::execute_skill_on_franka(run_loop* run_loop,
                                                 FrankaRobot* robot,
                                                 RobotStateData *robot_state_data) {
  double time = 0.0;
  int log_counter = 0;

  RunLoopSharedMemoryHandler* shared_memory_handler = run_loop->get_shared_memory_handler();
  RunLoopProcessInfo* run_loop_info = shared_memory_handler->getRunLoopProcessInfo();
  boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(
                                  *(shared_memory_handler->getRunLoopProcessInfoMutex()),
                                  boost::interprocess::defer_lock);
  SensorDataManager* sensor_data_manager = run_loop->get_sensor_data_manager();

  PoseTrajectoryGenerator* pose_trajectory_generator = dynamic_cast<PoseTrajectoryGenerator*>(traj_generator_);
  SetInternalImpedanceFeedbackController* internal_feedback_controller = dynamic_cast<SetInternalImpedanceFeedbackController*>(feedback_controller_);

  if (pose_trajectory_generator == nullptr) {
    throw std::bad_cast();
  }

  std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)>
      cartesian_pose_callback = [&](
      const franka::RobotState& robot_state,
      franka::Duration period) -> franka::CartesianPose {

    current_period_ = period.toSec();
    time += current_period_;

    if (time == 0.0) {
      pose_trajectory_generator->initialize_trajectory(robot_state, SkillType::CartesianPoseSkill);
      try {
        if (lock.try_lock()) {
          run_loop_info->set_time_skill_started_in_robot_time(robot_state.time.toSec());
          lock.unlock();
        } 
      } catch (boost::interprocess::lock_exception) {
        // Do nothing
      }
      for(int i = 0; i < 3; i++){
        current_position_[i] = robot_state.O_T_EE_d[12+i];
      }
    }
    traj_generator_->time_ = time;
    traj_generator_->dt_ = current_period_;

    try {
      sensor_data_manager->getSensorBufferGroupMutex()->try_lock();
      traj_generator_->parse_sensor_data(robot_state);
      termination_handler_->parse_sensor_data(robot_state);
      sensor_data_manager->getSensorBufferGroupMutex()->unlock();
    } catch (boost::interprocess::lock_exception) {
    }
    
    if (time > 0.0) {
      traj_generator_->get_next_step(robot_state);
    }

    bool done = termination_handler_->should_terminate(robot_state, model_, traj_generator_);

    std::array<double, 16> desired_pose = pose_trajectory_generator->get_desired_pose();
    std::array<double, 16> limited_desired_pose = desired_pose;
    if(current_period_ > 0.0) {
      limited_desired_pose = limit_position(desired_pose, current_period_);
    }

    log_counter += 1;
    if (log_counter % 1 == 0) {
      robot_state_data->log_robot_state(desired_pose, robot_state, robot->getModel(), time);
    } 

    if(time > 0.0 && done) {
      
      if(current_velocity_[0] != 0.0 || current_velocity_[1] != 0.0 || current_velocity_[2] != 0.0 ||
         current_acceleration_[0] != 0.0 || current_acceleration_[1] != 0.0 || current_acceleration_[2] != 0.0) {

        limited_desired_pose = limit_position_to_stop(previous_desired_pose_, current_period_);

        for(int i = 0; i < 3; i++) {
          previous_position_[i] = current_position_[i];
          previous_velocity_[i] = current_velocity_[i];
          previous_acceleration_[i] = current_acceleration_[i];

          current_position_[i] = limited_desired_pose[12+i];
          current_velocity_[i] = (current_position_[i] - previous_position_[i]) / current_period_;
          current_acceleration_[i] = (current_velocity_[i] - previous_velocity_[i]) / current_period_;
          current_jerk_[i] = (current_acceleration_[i] - previous_acceleration_[i]) / current_period_;
        }

        previous_desired_pose_ = limited_desired_pose;

        return limited_desired_pose;
      }

      try {
        if (lock.try_lock()) {
          run_loop_info->set_time_skill_finished_in_robot_time(robot_state.time.toSec());
          lock.unlock();
        } 
      } catch (boost::interprocess::lock_exception) {
        // Do nothing
      }
      
      return franka::MotionFinished(previous_desired_pose_);
    }

    if(current_period_ > 0.0) {
      for(int i = 0; i < 3; i++) {
        previous_position_[i] = current_position_[i];
        previous_velocity_[i] = current_velocity_[i];
        previous_acceleration_[i] = current_acceleration_[i];

        current_position_[i] = limited_desired_pose[12+i];
        current_velocity_[i] = (current_position_[i] - previous_position_[i]) / current_period_;
        current_acceleration_[i] = (current_velocity_[i] - previous_velocity_[i]) / current_period_;
        current_jerk_[i] = (current_acceleration_[i] - previous_acceleration_[i]) / current_period_;
      }
    }

    previous_desired_pose_ = limited_desired_pose;

    return limited_desired_pose;
  };

  if (internal_feedback_controller->set_cartesian_impedance_) {
    robot->robot_.control(cartesian_pose_callback, franka::ControllerMode::kCartesianImpedance);
  } else {
    robot->robot_.control(cartesian_pose_callback, franka::ControllerMode::kJointImpedance);
  }
}

