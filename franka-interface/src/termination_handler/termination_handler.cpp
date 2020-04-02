//
// Created by mohit on 11/25/18.
//

#include "franka-interface/termination_handler/termination_handler.h"

void TerminationHandler::check_terminate_preempt() {
  if (!done_) {
    done_ = run_loop_info_->get_skill_preempted();
  }
}

bool TerminationHandler::has_terminated() {
  return done_;
}

bool TerminationHandler::has_terminated_by_virt_coll() {
  return terminated_by_virt_coll_;
}

void TerminationHandler::check_terminate_time(TrajectoryGenerator *trajectory_generator) {
  if (!done_) {
    done_ = (trajectory_generator->time_ > trajectory_generator->run_time_ + buffer_time_);
  }
}

void TerminationHandler::check_terminate_virtual_wall_collisions(const franka::RobotState &robot_state, franka::Model *robot_model) {
  if (!done_) {
    int n_frame = 0;
    // Check all joints that are not base or EE
    for (franka::Frame frame = franka::Frame::kJoint2; frame <= franka::Frame::kFlange; frame++) {
      std::array<double, 16> pose = robot_model->pose(frame, robot_state);
      Eigen::Vector3d pos(pose[12], pose[13], pose[14]);
    
      for (uint n_plane = 0; n_plane < planes_.size(); n_plane++) {
        double dist = planes_[n_plane].absDistance(pos);
        
        if (dist < dist_thresholds_[n_frame]) {
          std::cout << "Frame " << n_frame + 1 << "is in collision with wall" << n_plane << "with distance " << dist << std::endl;
          done_ = true;
          terminated_by_virt_coll_ = true;
          break;
        }
      }

      n_frame++;
      if (done_) { break; }
    }
  }
}

void TerminationHandler::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readTerminationHandlerSensorMessage(should_terminate_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
    done_ = should_terminate_msg_.should_terminate();
  }
}