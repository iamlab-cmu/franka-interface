//
// Created by Ruthrash Hari on 7/9/23
//
#include "franka-interface/skills/joint_torque_skill.h"
#include <franka/robot.h>

#include "franka-interface/robot_state_data.h"
#include "franka-interface/run_loop.h"
#include "franka-interface/run_loop_shared_memory_handler.h"
#include "franka-interface/feedback_controller/set_internal_impedance_feedback_controller.h"
#include "franka-interface/trajectory_generator/joint_trajectory_generator.h"
#include <franka-interface-common/run_loop_process_info.h>

////// unneccessary maybe?

void JointTorqueSkill::limit_current_joint_torques(double period) {
  for(int i = 0; i < 7; i++) {
    if(std::abs(current_joint_torques_[i]) > max_joint_torques_[i] * safety_factor) {
      if(current_joint_torques_[i] > 0) {
        current_joint_torques_[i] = max_joint_torques_[i] * safety_factor;
      } else {
        current_joint_torques_[i] = -max_joint_torques_[i] * safety_factor;
      }
    }
    current_joint_rotatums_[i] = (current_joint_torques_[i] - previous_joint_torques_[i]) / period;

    if(std::abs(current_joint_rotatums_[i]) > max_joint_rotatums_[i] * safety_factor) {
      if(current_joint_rotatums_[i] > 0) {
        current_joint_rotatums_[i] = max_joint_rotatums_[i] * safety_factor;
      } else {
        current_joint_rotatums_[i] = -max_joint_rotatums_[i] * safety_factor;
      }
    }
    current_joint_torques_[i] = previous_joint_torques_[i] + current_joint_rotatums_[i] * period;
  }
}
void JointTorqueSkill::execute_skill_on_franka(run_loop* run_loop,
                                                FrankaRobot* robot,
                                                FrankaGripper* gripper,
                                                RobotStateData *robot_state_data) {
    double time = 0.0; 
    int log_counter = 0; 
    std::array<double, 16> pose_desired; 

    RunLoopSharedMemoryHandler* shared_memory_handler = run_loop->get_shared_memory_handler();
    RunLoopProcessInfo* run_loop_info = shared_memory_handler->getRunLoopProcessInfo();
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(
                                    *(shared_memory_handler->getRunLoopProcessInfoMutex()),
                                    boost::interprocess::defer_lock);
    SensorDataManager* sensor_data_manager = run_loop->get_sensor_data_manager();

    std::cout << "Will run the control loop\n";
    //Unneccessary for torque skill
    //JointTrajectoryGenerator* joint_trajectory_generator = dynamic_cast<JointTrajectoryGenerator*>(traj_generator_);
    //SetInternalImpedanceFeedbackController* internal_feedback_controller = dynamic_cast<SetInternalImpedanceFeedbackController*>(feedback_controller_);

    //if(joint_trajectory_generator == nullptr) {
    //    throw std::bad_cast();
    //}

    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        joint_torque_callback = [&](
        const franka::RobotState& robot_state,
        franka::Duration period) -> franka::Torques {
	    current_period_ = period.toSec();
            time += current_period_;        
            if(time == 0.0 ){
                //////
                //traj_generator_->initialize_trajectory(robot_state, SkillType::JointTorqueSkill);
                for(int i = 0; i < 7; i++) {
                    current_joint_torques_[i] = 0;
                    previous_joint_torques_[i] = 0;
                }

                try {
                    if (lock.try_lock()) {
                    run_loop_info->set_time_skill_started_in_robot_time(robot_state.time.toSec());
                    lock.unlock();
                    } 
                } catch (boost::interprocess::lock_exception) {
                    // Do nothing
                    }                
            }

            if (robot_state_data->mutex_.try_lock()) {
            robot_state_data->counter_ += 1;
            robot_state_data->time_ =  period.toSec();
            robot_state_data->has_data_ = true;
            robot_state_data->mutex_.unlock();
            }

            //traj_generator_->time_ = time;//////
            //traj_generator_->dt_ = current_period_;//////

            log_counter += 1;            
            try {
                sensor_data_manager->getSensorBufferGroupMutex()->try_lock();
                traj_generator_->parse_sensor_data(robot_state);
                termination_handler_->parse_sensor_data(robot_state);
                sensor_data_manager->getSensorBufferGroupMutex()->unlock();
                } catch (boost::interprocess::lock_exception) {
                    }
            log_counter += 1;
            if (log_counter % 1 == 0) {
            pose_desired = robot_state.O_T_EE_d;
            robot_state_data->log_robot_state(pose_desired, robot_state, robot->getModel(), time);
            }            

            //if (time > 0.0) {//////
            //    traj_generator_->get_next_step(robot_state);
            //}
            feedback_controller_->parse_sensor_data(robot_state);
            feedback_controller_->get_next_step(robot_state, traj_generator_);
            bool done = termination_handler_->should_terminate(robot_state, model_, traj_generator_);
            if (done && time > 0.0) {            
                try{
                    if (lock.try_lock()) {
                    run_loop_info->set_time_skill_finished_in_robot_time(robot_state.time.toSec());
                    lock.unlock();
                    } 
                }catch (boost::interprocess::lock_exception) {
        // Do nothing
                }
                return franka::MotionFinished(franka::Torques(feedback_controller_->tau_d_array_));
            }
            for(int i = 0; i < 7; i++) {
                current_joint_torques_[i] = feedback_controller_->tau_d_array_[i];
            }

            if (period.toSec() > 0.0){ 
            limit_current_joint_torques(current_period_);     
            }

            for(int i = 0; i < 7; i++) {
                previous_joint_torques_[i] = current_joint_torques_[i];
            }
            return current_joint_torques_;   
    
    };
    robot->robot_.control(joint_torque_callback, true);    
}
