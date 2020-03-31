#include "franka-interface/skills/gripper_skill.h"

#include <cassert>

#include "franka-interface/run_loop.h"
#include "franka-interface/run_loop_shared_memory_handler.h"
#include "franka-interface/trajectory_generator/gripper_trajectory_generator.h"

#include <franka-interface-common/run_loop_process_info.h>

void GripperSkill::execute_skill_on_franka(run_loop* run_loop,
                                           FrankaRobot *robot,
                                           RobotStateData *robot_state_data) {
  RunLoopSharedMemoryHandler* shared_memory_handler = run_loop->get_shared_memory_handler();
  RunLoopProcessInfo* run_loop_info = shared_memory_handler->getRunLoopProcessInfo();
  boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(
                            *(shared_memory_handler->getRunLoopProcessInfoMutex()),
                            boost::interprocess::defer_lock);

  franka::GripperState gripper_state = robot->getGripperState();
  franka::RobotState robot_state = robot->getRobotState();

  try {
    if (lock.try_lock()) {
      run_loop_info->set_time_skill_started_in_robot_time(robot_state.time.toSec());
      lock.unlock();
    } 
  } catch (boost::interprocess::lock_exception) {
    // Do nothing
  }

  // Check for the maximum grasping width.
  
  GripperTrajectoryGenerator *gripper_traj_generator = static_cast<
      GripperTrajectoryGenerator *>(traj_generator_);
  double desired_gripper_width = gripper_traj_generator->getWidth();
  if (gripper_state.max_width < desired_gripper_width) {
    std::cout << "Object is too large for the current fingers on the gripper." << std::endl;
    return_status_ = false;
    return;
  }

  double desired_gripper_speed = gripper_traj_generator->getSpeed();
  if (gripper_traj_generator->isGraspSkill()) {
    // TOOD(Mohit): Maybe stop the gripper before trying to grip again?
    franka::GripperState gripper_state = robot->getGripperState();
    if (!gripper_state.is_grasped) {
      return_status_ = robot->gripper_.grasp(desired_gripper_width, 
                      desired_gripper_speed, gripper_traj_generator->getForce(),
                      0.1, 0.1);
    }
  } else {
    return_status_ = robot->gripper_.move(desired_gripper_width, desired_gripper_speed);
  }

  gripper_state = robot->getGripperState();
  robot_state = robot->getRobotState();
  try {
    if (lock.try_lock()) {
      run_loop_info->set_time_skill_finished_in_robot_time(robot_state.time.toSec());
      lock.unlock();
    } 
  } catch (boost::interprocess::lock_exception) {
  // Do nothing
  }

  termination_handler_->done_ = true;

}

bool GripperSkill::has_terminated(FrankaRobot *robot){
  // Wait for some time before terminating this skill.
  return true;
}
