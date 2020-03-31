//
// Created by iamlab on 12/2/18.
//
#include "franka-interface/skills/base_skill.h"

int BaseSkill::get_skill_id() {
  return skill_idx_;
}

int BaseSkill::get_meta_skill_id() {
  return meta_skill_idx_;
}

std::string BaseSkill::get_description() {
  return description_;
}

void BaseSkill::set_skill_status(SkillStatus status) {
  // TODO(Mohit): Maybe add checks such that task status progresses
  // in one direction.
  skill_status_ = status;
}

SkillStatus BaseSkill::get_current_skill_status() {
  return skill_status_;
}

TrajectoryGenerator* BaseSkill::get_trajectory_generator() {
  return traj_generator_;
}

FeedbackController* BaseSkill::get_feedback_controller() {
  return feedback_controller_;
}

TerminationHandler* BaseSkill::get_termination_handler() {
  return termination_handler_;
}

void BaseSkill::start_skill(FrankaRobot* robot,
                            TrajectoryGenerator* traj_generator,
                            FeedbackController* feedback_controller,
                            TerminationHandler* termination_handler) {
  skill_status_ = SkillStatus::TO_START;
  traj_generator_ = traj_generator;
  feedback_controller_ = feedback_controller;
  termination_handler_ = termination_handler;
  termination_handler_->initialize_handler(robot);
  feedback_controller_->initialize_controller(robot);
  model_ = robot->getModel();
}

bool BaseSkill::has_terminated(FrankaRobot* robot) {
  return termination_handler_->has_terminated();
}

bool BaseSkill::has_terminated_by_virt_coll() {
  return termination_handler_->has_terminated_by_virt_coll();
}

void BaseSkill::write_result_to_shared_memory(SharedBufferTypePtr result_buffer, FrankaRobot* robot) {
  std::cout << "Should write result to shared memory\n";
}

void BaseSkill::write_feedback_to_shared_memory(SharedBufferTypePtr feedback_buffer) {
  std::cout << "Should write feedback to shared memory\n";
}

