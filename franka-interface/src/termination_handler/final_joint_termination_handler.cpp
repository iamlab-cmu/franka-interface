//
// Created by mohit on 11/30/18.
//

#include "franka-interface/termination_handler/final_joint_termination_handler.h"

#include <exception>

#include "franka-interface/trajectory_generator/joint_trajectory_generator.h"

void FinalJointTerminationHandler::parse_parameters() {
  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = joint_termination_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    buffer_time_ = joint_termination_params_.buffer_time();

    if (joint_termination_params_.joint_thresholds_size() == 7) {
      for (int i = 0; i < 7; i++) {
        joint_thresholds_[i] = joint_termination_params_.joint_thresholds(i);
      }
    } else {
      for (int i = 0; i < 7; i++) {
        joint_thresholds_[i] = default_joint_threshold_;
      }
    }
  } else {
    std::cout << "Parsing FinalJointTerminationHandler params failed. Data size = " << data_size << std::endl;
  }
}

bool FinalJointTerminationHandler::should_terminate(const franka::RobotState &robot_state, 
                                                              franka::Model *model,
                                                              TrajectoryGenerator *trajectory_generator) {

  check_terminate_virtual_wall_collisions(robot_state, model);
  check_terminate_preempt();
  check_terminate_time(trajectory_generator);

  if (!done_) {
    JointTrajectoryGenerator *joint_traj_generator =
        dynamic_cast<JointTrajectoryGenerator *>(trajectory_generator);

    if(joint_traj_generator == nullptr) {
      throw std::bad_cast();
    }

    std::array<double, 7> desired_joints = joint_traj_generator->get_desired_joints();
    std::array<double, 7> goal_joints = joint_traj_generator->get_goal_joints();

    for(size_t i = 0; i < goal_joints.size(); i++) {
      if(fabs(goal_joints[i] - desired_joints[i]) > joint_thresholds_[i]) {
        return false;
      }
    }

    done_ = true;
  }
  
  return done_;
}

