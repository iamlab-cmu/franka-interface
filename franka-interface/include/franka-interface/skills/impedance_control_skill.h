#ifndef FRANKA_INTERFACE_SKILLS_IMPEDANCE_CONTROL_SKILL_H_
#define FRANKA_INTERFACE_SKILLS_IMPEDANCE_CONTROL_SKILL_H_

#include "franka-interface/skills/base_skill.h"

class ImpedanceControlSkill : public BaseSkill {
 public:
  ImpedanceControlSkill(int skill_idx, int meta_skill_idx, std::string description) : 
            BaseSkill(skill_idx, meta_skill_idx, description) 
  {};

  void execute_skill_on_franka(run_loop* run_loop, 
                               FrankaRobot* robot, 
                               FrankaGripper* gripper,
                               RobotStateData* robot_state_data) override;

 protected:
  const std::array<double, 7> k_gains_ = {{600.0, 600.0, 600.0, 600.0, 
                                           250.0, 150.0, 50.0}};
  // Damping
  const std::array<double, 7> d_gains_ = {{50.0, 50.0, 50.0, 50.0, 
                                           30.0, 25.0, 15.0}};
};

#endif  // FRANKA_INTERFACE_SKILLS_IMPEDANCE_CONTROL_SKILL_H_