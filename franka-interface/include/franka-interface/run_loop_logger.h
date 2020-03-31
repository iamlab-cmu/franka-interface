#ifndef FRANKA_INTERFACE_RUN_LOOP_LOGGER_H_
#define FRANKA_INTERFACE_RUN_LOOP_LOGGER_H_

#include <vector>
#include <string>
#include <mutex>

enum RunLoopLoggerCode {INFO, WARNING, ERROR, ABORT};

class RunLoopLogger {
 public:
  RunLoopLogger(std::mutex& m) : mutex_(m) {};

  std::mutex& mutex_;

  void add_log(std::string log, RunLoopLoggerCode code);

  void add_error_log(std::string log);

  void add_warning_log(std::string log);

  void add_info_log(std::string log);

  void print_error_log();

  void print_warning_log();

  void print_info_log();

 private:
  std::vector<std::string> error_logs_={};
  std::vector<std::string> warning_logs_={};
  std::vector<std::string> info_logs_={};

  void print_error_logs_to_console(std::vector<std::string>& logs);
};

#endif  // FRANKA_INTERFACE_RUN_LOOP_LOGGER_H_