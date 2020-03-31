#include <franka-interface-common/franka_interface_state_info.h>

#include <cstring>

bool FrankaInterfaceStateInfo::get_is_ready() {
  return is_ready_;
}

void FrankaInterfaceStateInfo::set_is_ready(bool is_ready) {
  is_ready_ = is_ready;
}

int FrankaInterfaceStateInfo::get_watchdog_counter() {
  return watchdog_counter_;
}

void FrankaInterfaceStateInfo::reset_watchdog_counter() {
  watchdog_counter_ = 0;
}

void FrankaInterfaceStateInfo::increment_watchdog_counter() {
  watchdog_counter_++;
}

std::string FrankaInterfaceStateInfo::get_error_description() {
  std::string desc(error_description_,
                   error_description_ + sizeof(error_description_[0]) * error_description_len_);
  return desc;
}

void FrankaInterfaceStateInfo::set_error_description(std::string description){  
  std::memcpy(&error_description_, description.c_str(), description.size());
  error_description_len_  = description.size();
}

void FrankaInterfaceStateInfo::clear_error_description() {  
  set_error_description("");
}