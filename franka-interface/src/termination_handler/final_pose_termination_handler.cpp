//
// Created by mohit on 11/29/18.
//

#include "franka-interface/termination_handler/final_pose_termination_handler.h"

#include <exception>
#include <Eigen/Dense>

#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"

void FinalPoseTerminationHandler::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = pose_termination_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    buffer_time_ = pose_termination_params_.buffer_time();

    if (pose_termination_params_.position_thresholds_size() == 3) {
      for (int i = 0; i < 3; i++) {
        position_thresholds_[i] = pose_termination_params_.position_thresholds(i);
      }
    } else {
      for (int i = 0; i < 3; i++) {
        position_thresholds_[i] = default_position_threshold_;
      }
    }

    if (pose_termination_params_.orientation_thresholds_size() == 3) {
      for (int i = 0; i < 3; i++) {
        orientation_thresholds_[i] = pose_termination_params_.orientation_thresholds(i);
      }
    } else {
      for (int i = 0; i < 3; i++) {
        orientation_thresholds_[i] = default_orientation_threshold_;
      }
    }
  } else {
    std::cout << "Parsing FinalPoseTerminationHandler params failed. Data size = " << data_size << std::endl;
  }
}

bool FinalPoseTerminationHandler::should_terminate(const franka::RobotState &robot_state, 
                                                            franka::Model *model,
                                                             TrajectoryGenerator *trajectory_generator) {
  check_terminate_preempt();
  check_terminate_virtual_wall_collisions(robot_state, model);
  check_terminate_time(trajectory_generator);

  if(!done_){
    PoseTrajectoryGenerator *pose_trajectory_generator =
          dynamic_cast<PoseTrajectoryGenerator *>(trajectory_generator);

    if(pose_trajectory_generator == nullptr) {
      throw std::bad_cast();
    }

    // Terminate immediately if collision is detected
    std::array<double, 6> cartesian_collision = robot_state.cartesian_collision;

    for(int i = 0; i < 6; i++) {
      if(cartesian_collision[i] != 0) {
        done_ = true;
        return true;
      }
    }

    Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d current_position(current_transform.translation());
    Eigen::Quaterniond current_orientation(current_transform.linear());

    Eigen::Vector3d position_error = pose_trajectory_generator->get_goal_position() - current_position;
    
    Eigen::Quaterniond goal_orientation(pose_trajectory_generator->get_goal_orientation());

    if (goal_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0) {
      current_orientation.coeffs() << -current_orientation.coeffs();
    }

    Eigen::Quaterniond error_quaternion(current_orientation * goal_orientation.inverse());
    // convert to axis angle
    Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
    // compute "orientation error"
    Eigen::Vector3d orientation_error = error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
    
    for(int i=0; i<3; i++) {
      if(std::abs(position_error[i]) > position_thresholds_[i] || 
         std::abs(orientation_error[i]) > orientation_thresholds_[i]) {
        return false;
      }
    }

    done_ = true;

  }

  return done_;
}

