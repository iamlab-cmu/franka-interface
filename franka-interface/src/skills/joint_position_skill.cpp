//
// Created by mohit on 12/6/18.
//

#include "franka-interface/skills/joint_position_skill.h"

#include <cassert>

#include <franka/robot.h>

#include "franka-interface/robot_state_data.h"
#include "franka-interface/run_loop.h"
#include "franka-interface/run_loop_shared_memory_handler.h"
#include "franka-interface/feedback_controller/set_internal_impedance_feedback_controller.h"
#include "franka-interface/trajectory_generator/joint_trajectory_generator.h"

#include <franka-interface-common/run_loop_process_info.h>

void JointPositionSkill::execute_skill_on_franka(run_loop* run_loop,
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

  JointTrajectoryGenerator* joint_trajectory_generator = dynamic_cast<JointTrajectoryGenerator*>(traj_generator_);
  SetInternalImpedanceFeedbackController* internal_feedback_controller = dynamic_cast<SetInternalImpedanceFeedbackController*>(feedback_controller_);

  if(joint_trajectory_generator == nullptr) {
    throw std::bad_cast();
  }

  std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)>
      joint_pose_callback = [&](
      const franka::RobotState& robot_state,
      franka::Duration period) -> franka::JointPositions {

    current_period_ = period.toSec();
    time += current_period_;

    if (time == 0.0) {
      joint_trajectory_generator->initialize_trajectory(robot_state, SkillType::JointPositionSkill);
      try {
        if (lock.try_lock()) {
          run_loop_info->set_time_skill_started_in_robot_time(robot_state.time.toSec());
          lock.unlock();
        } 
      } catch (boost::interprocess::lock_exception) {
        // Do nothing
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
    
    if(time > 0.0) {
      traj_generator_->get_next_step(robot_state);
    }

    bool done = termination_handler_->should_terminate(robot_state, model_,
                                                                 traj_generator_);
    franka::JointPositions joint_desired(joint_trajectory_generator->get_desired_joints());

    log_counter += 1;
    if (log_counter % 1 == 0) {
      pose_desired = robot_state.O_T_EE_d;
      robot_state_data->log_robot_state(pose_desired, robot_state, robot->getModel(), time);
    }
    
    if (done && time > 0.0) {
      try {
        if (lock.try_lock()) {
          run_loop_info->set_time_skill_finished_in_robot_time(robot_state.time.toSec());
          lock.unlock();
        } 
      } catch (boost::interprocess::lock_exception) {
        // Do nothing
      }
      return franka::MotionFinished(joint_desired);
    }

    return joint_desired;
  };

  if (internal_feedback_controller->set_cartesian_impedance_) {
    robot->robot_.control(joint_pose_callback, franka::ControllerMode::kCartesianImpedance);
  } else {
    robot->robot_.control(joint_pose_callback, franka::ControllerMode::kJointImpedance);
  }
}

