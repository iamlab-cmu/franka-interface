#pragma once

#include <cassert>
#include <string>

class FrankaInterfaceStateInfo {
 public:
  FrankaInterfaceStateInfo(){};

  bool get_is_ready();
  void set_is_ready(bool is_ready);

  int get_watchdog_counter();
  void reset_watchdog_counter(); // to be used by franka_interface
  void increment_watchdog_counter(); // to be used by franka_ros_interface

  std::string get_error_description();
  void set_error_description(std::string description); // to be used by franka_interface
  void clear_error_description(); // to be used by franka_ros_interface

 private:
  bool is_ready_{false};
  int watchdog_counter_{0};
  char error_description_[1000];
  size_t error_description_len_=0;
};

