//
// Created by mohit on 3/23/19.
//

#ifndef FRANKA_INTERFACE_LOGGER_UTILS_H
#define FRANKA_INTERFACE_LOGGER_UTILS_H

#include <string>
#include <vector>

class LoggerUtils {
 public:
  static std::vector<std::string> all_logger_files(std::string logdir);

  static int integer_suffix_for_new_log_file(std::string logdir);
};

#endif //FRANKA_INTERFACE_LOGGER_UTILS_H
