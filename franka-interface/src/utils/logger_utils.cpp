//
// Created by mohit on 3/23/19.
//

#include "franka-interface/utils/logger_utils.h"

#include <boost/filesystem.hpp>


std::vector<std::string> LoggerUtils::all_logger_files(std::string logdir) {
  std::string prefix = "robot_state_data_";
  std::vector<std::string> all_log_files;
  for (boost::filesystem::directory_entry& f : boost::filesystem::directory_iterator(logdir)) {
    std::string filename = f.path().filename().string();
    if (filename.find(prefix) != std::string::npos) {
      all_log_files.push_back(filename);
    }
  }
  return all_log_files;
}

int LoggerUtils::integer_suffix_for_new_log_file(std::string logdir) {
  std::string prefix = "robot_state_data_";
  int curr_suffix = 0;
  for (boost::filesystem::directory_entry& f : boost::filesystem::directory_iterator(logdir)) {
    std::string filename = f.path().filename().string();
    if (filename.find(prefix) != std::string::npos) {
      int int_suffix = std::stoi(filename.substr(prefix.length(), filename.find(".txt")));
      if (int_suffix >= curr_suffix) {
        curr_suffix = int_suffix + 1;
      }
    }
  }
  return curr_suffix;
}
