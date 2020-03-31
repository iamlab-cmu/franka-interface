#include "franka-interface/skills/joint_position_dynamic_interp_skill.h"

#include <cassert>

#include <franka/robot.h>

#include "franka-interface/robot_state_data.h"
#include "franka-interface/run_loop.h"
#include "franka-interface/run_loop_shared_memory_handler.h"
#include "franka-interface/trajectory_generator/joint_trajectory_generator.h"

#include <franka-interface-common/run_loop_process_info.h>

void JointPositionDynamicInterpSkill::execute_skill_on_franka(
    run_loop* run_loop, FrankaRobot* robot, RobotStateData *robot_state_data) {

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

  JointSensorInfo joint_sensor_msg;

  if(joint_trajectory_generator == nullptr) {
    throw std::bad_cast();
  }

  bool did_receive_sensor_msg = false;
  std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)>
      joint_pose_callback = [&](
      const franka::RobotState& robot_state,
      franka::Duration period) -> franka::JointPositions {
    time += period.toSec();

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
    traj_generator_->dt_ = period.toSec();
    if(time > 0.0) {
      traj_generator_->get_next_step(robot_state);
    }

    bool done = termination_handler_->should_terminate(robot_state, model_, traj_generator_);
    franka::JointPositions joint_desired(joint_trajectory_generator->get_desired_joints());

    log_counter += 1;
    if (log_counter % 1 == 0) {
      pose_desired = robot_state.O_T_EE_d;
      robot_state_data->log_robot_state(pose_desired, robot_state, model_, time);
    }

    JointSensorInfo new_joint_sensor_info;

    SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager->readJointSensorInfoMessage(
        new_joint_sensor_info);
    if (!did_receive_sensor_msg && sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
      did_receive_sensor_msg = true;
      assert(new_joint_sensor_info.IsInitialized());
      std::array<double, 7> new_goal_joints = {
          new_joint_sensor_info.q1(),
          new_joint_sensor_info.q2(),
          new_joint_sensor_info.q3(),
          new_joint_sensor_info.q4(),
          new_joint_sensor_info.q5(),
          new_joint_sensor_info.q6(),
          new_joint_sensor_info.q7(),
      };
      joint_trajectory_generator->setGoalJoints(new_goal_joints);
      std::cout << "Updated new goal joints: ";
      for (int i = 0; i < new_goal_joints.size(); i++) {
        std::cout << new_goal_joints[i] << ", ";
      }
      std::cout << std::endl;
      // HACK: Reset the time manually. This is bad because we should not manually set this here.
      joint_trajectory_generator->setInitialJoints(robot_state.q_d);
      // DEBUG
      std::cout << "q" << std::endl;
      for (int i = 0; i < 7; i++) {
        std::cout << robot_state.q[i] << ", ";
      }
      std::cout << std::endl;

      std::cout << "q_d" << std::endl;
      for (int i = 0; i < 7; i++) {
        std::cout << robot_state.q_d[i] << ", ";
      }
      std::cout << std::endl;

      // We have to set time to 0, to smoothly change positions. In an idealized world there is a
      // better way to do this, i.e., we assume that there is a non-zero joint velocity at the
      // start of the robot motion and we want to smoothly interpolate it to the target location
      // with 0 target velocity.
      // We do something similar for our contacts based stuff.
      time = 0.0;
      traj_generator_->time_ = 0.0;
    }

    // Set the desired joints manually

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

  robot->robot_.control(joint_pose_callback);
}

