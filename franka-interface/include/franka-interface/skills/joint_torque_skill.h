
#ifndef FRANKA_INTERFACE_SKILLS_JOINT_TORQUE_SKILL_H_
#define FRANKA_INTERFACE_SKILLS_JOINT_TORQUE_SKILL_H_

#include "franka-interface/skills/base_skill.h"

class JointTorqueSkill: public BaseSkill{
    public:
        JointTorqueSkill(int skill_idx, int meta_skill_idx, std::string description):
                                    BaseSkill(skill_idx, meta_skill_idx, description)
        {}; 
        void limit_current_joint_torques(double period);        
        void execute_skill_on_franka(run_loop* run_loop,
                                    FrankaRobot* robot, 
                                    FrankaGripper* gripper,
                                    RobotStateData* robot_state_data) override;
    private:
        bool return_status_{false};

        double safety_factor = 0.1;

        std::array<double, 7> previous_joint_torques_;
        std::array<double, 7> current_joint_torques_;
        std::array<double, 7> current_joint_rotatums_;

        // Franka Parameters from https://frankaemika.github.io/docs/control_parameters.html
        std::array<double, 7> max_joint_torques_{87, 87, 87, 87, 12, 12, 12}; // Nm
        std::array<double, 7> max_joint_rotatums_{1000, 1000, 1000, 1000, 1000, 1000, 1000}; // Nm / s        
        }; 
#endif // FRANKA_INTERFACE_SKILLS_JOINT_TORQUE_SKILL_H_