#include "franka-interface/run_loop_logger.h"

#include <cassert>
#include <iostream>

void RunLoopLogger::add_log(std::string log, RunLoopLoggerCode code) {
  if (code == RunLoopLoggerCode::INFO) {
    add_info_log(log);
  } else if (code == RunLoopLoggerCode::ERROR) {
    add_error_log(log);
  } else if (code == RunLoopLoggerCode::WARNING) {
    add_warning_log(log);
  } else if (code == RunLoopLoggerCode::ABORT) {
    // pass
  } else {
    // pass
  }
}

/**
 * TODO(Mohit): Add timestamps to all of these.
 */

void RunLoopLogger::add_error_log(std::string log) {
  if (mutex_.try_lock()) {
    error_logs_.push_back(log);
    mutex_.unlock();
  }
}

void RunLoopLogger::add_warning_log(std::string log) {
  if (mutex_.try_lock()) {
    warning_logs_.push_back(log);
    mutex_.unlock();
  }
}

void RunLoopLogger::add_info_log(std::string log) {
  if (mutex_.try_lock()) {
    info_logs_.push_back(log);
    mutex_.unlock();
  }
}

void RunLoopLogger::print_error_logs_to_console(std::vector<std::string>& logs) {
  if (mutex_.try_lock()) {
    for (auto it = logs.begin(); it != logs.end(); it++) {
      std::cout << *it << "\n";
    }
    logs.clear();
    mutex_.unlock();
  }
}

void RunLoopLogger::print_error_log() {
  print_error_logs_to_console(error_logs_);
}

void RunLoopLogger::print_warning_log() {
  print_error_logs_to_console(warning_logs_);
}

void RunLoopLogger::print_info_log() {
  print_error_logs_to_console(info_logs_);
}
