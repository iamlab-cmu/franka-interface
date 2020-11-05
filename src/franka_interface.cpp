#include <iostream>
#include <mutex>
#include <boost/program_options.hpp>

#include <franka-interface-common/definitions.h>

#include <franka-interface/run_loop.h>

namespace po = boost::program_options;

int main(int argc, char *argv[]) {

  try {
    bool stop_franka_interface_on_error;
    bool reset_skill_numbering_on_error;
    bool use_new_filestream_on_error;
    bool with_gripper;
    bool log;
    std::string logdir;
    std::string robot_ip;
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "Produce help message")
      ("robot_ip,ip_addr,ip", po::value<std::string>(&robot_ip)->default_value("172.16.0.2"),
            "Robot's ip address")
      ("stop_on_error", po::value<bool>(&stop_franka_interface_on_error)->default_value(false),
            "Stop robo-lib on error, i.e. any exception thrown by libfranka.")
      ("reset_skill_numbering_on_error", po::value<bool>(&reset_skill_numbering_on_error)->default_value(false),
            "Reset skill numbering on error, i.e. any exception thrown by libfranka.")
      ("use_new_filestream_on_error", po::value<bool>(&use_new_filestream_on_error)->default_value(false),
            "Use a new filestream on error, i.e. any exception thrown by libfranka.")
      ("log", po::value<bool>(&log)->default_value(false), "Log at 1kHz in Franka-Interface")
      ("logdir", po::value<std::string>(&logdir)->default_value("logs"), "Directory to save robot_state_data")
      ("with_gripper", po::value<bool>(&with_gripper)->default_value(true), "Robot has gripper attached")    
    ;

    po::positional_options_description p;
    p.add("robot_ip", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
              options(desc).positional(p).run(), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << "Usage: options_description [options]\n";
        std::cout << desc;
        return 0;
    }

    std::cout << "IAM FrankaInterface\n";
    std::mutex m;
    std::mutex robot_loop_data_mutex;
    run_loop rl = run_loop(std::ref(m), std::ref(robot_loop_data_mutex), robot_ip,
        stop_franka_interface_on_error, reset_skill_numbering_on_error, use_new_filestream_on_error, log, logdir, with_gripper);
    std::cout << "Will start run loop.\n";
    
    rl.start();
    std::cout << "Did start run loop.\n";
    std::cout << "Will run..\n";
    rl.run_on_franka();
    
  }
  catch(std::exception& e) {
    std::cout << e.what() << "\n";
    return 1;
  }
  
  return 0;
}
