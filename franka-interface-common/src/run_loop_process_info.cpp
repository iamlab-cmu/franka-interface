//
// Created by mohit on 11/20/18.
//

#include <franka-interface-common/run_loop_process_info.h>

#include <cstring>

int RunLoopProcessInfo::get_current_shared_memory_index() {
  return current_memory_region_;
}

int RunLoopProcessInfo::get_current_free_shared_memory_index() {
  int current_free_memory_idx = 1 - current_memory_region_;
  return current_free_memory_idx;
}

void RunLoopProcessInfo::update_shared_memory_region() {
  assert(current_memory_region_ == 0 or current_memory_region_ == 1);
  current_memory_region_ = (current_memory_region_ + 1) % 2;
}

int RunLoopProcessInfo::get_current_shared_sensor_index() {
  return current_sensor_region_;
}

int RunLoopProcessInfo::get_current_free_shared_sensor_index() {
  int current_free_sensor_idx = 1 - current_sensor_region_;
  return current_free_sensor_idx;
}

void RunLoopProcessInfo::update_shared_sensor_region() {
  assert(current_sensor_region_ == 0 or current_sensor_region_ == 1);
  current_sensor_region_ = (current_sensor_region_ + 1) % 2;
}

int RunLoopProcessInfo::get_current_shared_feedback_index() {
  return current_feedback_region_;
}

int RunLoopProcessInfo::get_current_free_shared_feedback_index() {
  int current_free_feedback_idx = 1 - current_feedback_region_;
  return current_free_feedback_idx;
}

void RunLoopProcessInfo::update_shared_feedback_region() {
  assert(current_feedback_region_ == 0 or current_feedback_region_ == 1);
  current_feedback_region_ = (current_feedback_region_ + 1) % 2;
}

bool RunLoopProcessInfo::can_run_new_skill() {
  return is_running_skill_ == false;
}

int RunLoopProcessInfo::get_current_memory_region() {
  return current_memory_region_;
}

int RunLoopProcessInfo::get_current_sensor_region() {
  return current_sensor_region_;
}

int RunLoopProcessInfo::get_current_feedback_region() {
  return current_feedback_region_;
}

int RunLoopProcessInfo::get_current_skill_id() {
  return current_skill_id_;
}

void RunLoopProcessInfo::set_current_skill_id(int current_skill_id) {
  // Make sure we are updating to the latest available skill.
  assert(current_skill_id == new_skill_id_);
  current_skill_id_ = current_skill_id;
}

int RunLoopProcessInfo::get_current_skill_type() {
  return current_skill_type_;
}

void RunLoopProcessInfo::set_current_skill_type(int current_skill_type) {
  // Make sure we are updating to the latest available skill.
  assert(current_skill_type == new_skill_type_);
  current_skill_type_ = current_skill_type;
}

int RunLoopProcessInfo::get_current_meta_skill_id() {
  return current_meta_skill_id_;
}

void RunLoopProcessInfo::set_current_meta_skill_id(int current_meta_skill_id) {
  // Make sure we are updating to the latest available skill.
  assert(current_meta_skill_id == new_meta_skill_id_);
  current_meta_skill_id_ = current_meta_skill_id;
}

int RunLoopProcessInfo::get_current_meta_skill_type() {
  return current_meta_skill_type_;
}

void RunLoopProcessInfo::set_current_meta_skill_type(int current_meta_skill_type) {
  // Make sure we are updating to the latest available skill.
  assert(current_meta_skill_type == new_meta_skill_type_);
  current_meta_skill_type_ = current_meta_skill_type;
}

std::string RunLoopProcessInfo::get_current_skill_description() {
  std::string current_skill_description(current_skill_description_, current_skill_description_ + 
                                        sizeof(current_skill_description_[0]) * current_skill_description_len_);
  return current_skill_description;
}

void RunLoopProcessInfo::set_current_skill_description(std::string current_skill_description) {
  // Make sure we are updating to the latest available skill.
  // assert(strcmp(current_skill_description.c_str(), new_skill_description_) == 0);
  assert(current_skill_description.size() == new_skill_description_len_);
  
  std::memcpy(&current_skill_description_, current_skill_description.c_str(), current_skill_description.size());
  current_skill_description_len_  = current_skill_description.size();
}

bool RunLoopProcessInfo::get_new_skill_available() {
  return new_skill_available_;
}

void RunLoopProcessInfo::set_new_skill_available(bool new_skill_available) {
  new_skill_available_ = new_skill_available;
}

int RunLoopProcessInfo::get_new_skill_id() {
  return new_skill_id_;
}

void RunLoopProcessInfo::set_new_skill_id(int new_skill_id) {
  // Make sure we are getting the new skill
  assert(new_skill_id == current_skill_id_ + 1);
  new_skill_id_ = new_skill_id;
}

int RunLoopProcessInfo::get_new_skill_type() {
  return new_skill_type_;
}

void RunLoopProcessInfo::set_new_skill_type(int new_skill_type) {
  new_skill_type_ = new_skill_type;
}

int RunLoopProcessInfo::get_new_meta_skill_id() {
  return new_meta_skill_id_;
}

void RunLoopProcessInfo::set_new_meta_skill_id(int new_meta_skill_id) {
  new_meta_skill_id_ = new_meta_skill_id;
}

int RunLoopProcessInfo::get_new_meta_skill_type() {
  return new_meta_skill_type_;
}

void RunLoopProcessInfo::set_new_meta_skill_type(int new_meta_skill_type) {
  new_meta_skill_type_ = new_meta_skill_type;
}

std::string RunLoopProcessInfo::get_new_skill_description() {
  std::string new_skill_description(new_skill_description_, new_skill_description_ + 
                                    sizeof(new_skill_description_[0]) * new_skill_description_len_);
  return new_skill_description;
}
void RunLoopProcessInfo::set_new_skill_description(std::string new_skill_description) {
  std::memcpy(&new_skill_description_, new_skill_description.c_str(), new_skill_description.size());
  new_skill_description_len_  = new_skill_description.size();
}

bool RunLoopProcessInfo::get_is_running_skill() {
  return is_running_skill_;
}

void RunLoopProcessInfo::set_is_running_skill(bool is_running_skill) {
  is_running_skill_ = is_running_skill;
}

bool RunLoopProcessInfo::get_skill_preempted() {
  return skill_preempted_;
}

void RunLoopProcessInfo::set_skill_preempted(bool skill_preempted) {
  skill_preempted_ = skill_preempted;
}

int RunLoopProcessInfo::get_done_skill_id() {
  return done_skill_id_;
}

void RunLoopProcessInfo::set_done_skill_id(int done_skill_id) {
  // Make sure we are getting the done skill
  assert(done_skill_id == current_skill_id_);
  done_skill_id_ = done_skill_id;
}

int RunLoopProcessInfo::get_result_skill_id() {
  return result_skill_id_;
}

void RunLoopProcessInfo::set_result_skill_id(int result_skill_id) {
  // Make sure we are getting the result skill
  assert(result_skill_id == done_skill_id_ ||
         result_skill_id == (done_skill_id_ - 1));
  result_skill_id_ = result_skill_id;
}

double RunLoopProcessInfo::get_time_skill_started_in_robot_time() {
  return time_skill_started_in_robot_time_;
}

void RunLoopProcessInfo::reset_time_skill_started_in_robot_time() {
  time_skill_started_in_robot_time_ = 0.0;
}

void RunLoopProcessInfo::set_time_skill_started_in_robot_time(double robot_time) {
  time_skill_started_in_robot_time_ = robot_time;
}

double RunLoopProcessInfo::get_time_skill_finished_in_robot_time() {
  return time_skill_finished_in_robot_time_;
}

void RunLoopProcessInfo::reset_time_skill_finished_in_robot_time() {
  time_skill_finished_in_robot_time_ = 0.0;
}

void RunLoopProcessInfo::set_time_skill_finished_in_robot_time(double robot_time) {
  time_skill_finished_in_robot_time_ = robot_time;
}

void RunLoopProcessInfo::reset_skill_vars() {
  current_memory_region_ = 1;
  current_sensor_region_ = 1;
  current_feedback_region_ = 1;

  current_skill_id_ = -1;
  current_skill_type_ = 0;
  current_meta_skill_id_ = -1;
  current_meta_skill_type_ = 0;
  memset(current_skill_description_, 0, sizeof current_skill_description_);
  current_skill_description_len_ = 0;

  new_skill_available_ = false;

  new_skill_id_ = -1;
  new_skill_type_ = 0;
  new_meta_skill_id_ = -1;
  new_meta_skill_type_ = 0;
  memset(new_skill_description_, 0, sizeof new_skill_description_);
  new_skill_description_len_ = 0;

  is_running_skill_ = false;
  skill_preempted_ = false;
  
  done_skill_id_ = -1;
  result_skill_id_ = -1;
  
  time_skill_started_in_robot_time_ = 0.0;
  time_skill_finished_in_robot_time_ = 0.0;
}

void RunLoopProcessInfo::set_skill_done_when_error_occurs(int skill_id) {
  // TODO: Some day have a skill error flag to make it better instead of 
  // current hacky way of setting done skill id to the current skill id.

  is_running_skill_ = false;
  skill_preempted_ = false;
  
  done_skill_id_ = skill_id;
  result_skill_id_ = skill_id;
}