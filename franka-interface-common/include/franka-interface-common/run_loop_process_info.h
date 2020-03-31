#pragma once

#include <cassert>
#include <string>

class RunLoopProcessInfo {
 public:
  RunLoopProcessInfo(int memory_region_idx): current_memory_region_(memory_region_idx) {};

  /**
   * Memory index being used by the run loop.
   */
  int get_current_shared_memory_index();

  /**
   * Memory index being used by the actionlib.
   */
  int get_current_free_shared_memory_index();

  /**
   * Update shared memory region.
   */
  void update_shared_memory_region();

  /**
   * Sensor index being used by the run loop.
   */
  int get_current_shared_sensor_index();

  /**
   * Sensor index being used by the actionlib.
   */
  int get_current_free_shared_sensor_index();

  /**
   * Update shared sensor region.
   */
  void update_shared_sensor_region();

  /**
   * Feedback index being used by the run loop.
   */
  int get_current_shared_feedback_index();

  /**
   * Feedback index being used by the actionlib.
   */
  int get_current_free_shared_feedback_index();

  /**
   * Update shared feedback region.
   */
  void update_shared_feedback_region();

  bool can_run_new_skill();

  int get_current_memory_region();

  int get_current_sensor_region();

  int get_current_feedback_region();

  int get_current_skill_id();
  void set_current_skill_id(int current_skill_id);

  int get_current_skill_type();
  void set_current_skill_type(int current_skill_type);

  int get_current_meta_skill_id();
  void set_current_meta_skill_id(int current_meta_skill_id);

  int get_current_meta_skill_type();
  void set_current_meta_skill_type(int current_meta_skill_type);

  std::string get_current_skill_description();
  void set_current_skill_description(std::string current_skill_description);

  bool get_new_skill_available();
  void set_new_skill_available(bool new_skill_available);

  int get_new_skill_id();
  void set_new_skill_id(int new_skill_id);

  int get_new_skill_type();
  void set_new_skill_type(int new_skill_type);

  int get_new_meta_skill_id();
  void set_new_meta_skill_id(int new_meta_skill_id);

  int get_new_meta_skill_type();
  void set_new_meta_skill_type(int new_meta_skill_type);

  std::string get_new_skill_description();
  void set_new_skill_description(std::string new_skill_description);

  bool get_is_running_skill();
  void set_is_running_skill(bool is_running_skill);

  bool get_skill_preempted();
  void set_skill_preempted(bool skill_preempted);

  int get_done_skill_id();
  void set_done_skill_id(int done_skill_id);
  
  int get_result_skill_id();
  void set_result_skill_id(int result_skill_id);

  double get_time_skill_started_in_robot_time();
  void reset_time_skill_started_in_robot_time();
  void set_time_skill_started_in_robot_time(double robot_time);
  
  double get_time_skill_finished_in_robot_time();
  void reset_time_skill_finished_in_robot_time();
  void set_time_skill_finished_in_robot_time(double robot_time);

  // reset internal variables about skills to their default values. used by franka_interface for error recovery
  void reset_skill_vars();

  void set_skill_done_when_error_occurs(int skill_id);

 private:
  int current_memory_region_{1}; 
  int current_sensor_region_{1};
  int current_feedback_region_{1};

  int current_skill_id_{-1}; 
  int current_skill_type_{0};
  int current_meta_skill_id_{-1};
  int current_meta_skill_type_{0};
  char current_skill_description_[1000];
  size_t current_skill_description_len_=1;

  bool new_skill_available_{false};

  int new_skill_id_{-1};
  int new_skill_type_{0};
  int new_meta_skill_id_{-1};
  int new_meta_skill_type_{0};
  char new_skill_description_[1000];
  size_t new_skill_description_len_=1;

  bool is_running_skill_{false};
  bool skill_preempted_{false};
  
  int done_skill_id_{-1};
  int result_skill_id_{-1};
  
  double time_skill_started_in_robot_time_{0.0};
  double time_skill_finished_in_robot_time_{0.0};
};

