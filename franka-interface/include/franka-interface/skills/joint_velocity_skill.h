#ifndef FRANKA_INTERFACE_SKILLS_JOINT_VELOCITY_SKILL_H_
#define FRANKA_INTERFACE_SKILLS_JOINT_VELOCITY_SKILL_H_

#include "franka-interface/skills/base_skill.h"

class JointVelocitySkill : public BaseSkill {
 public:
  JointVelocitySkill(int skill_idx, int meta_skill_idx, std::string description) : 
                              BaseSkill(skill_idx, meta_skill_idx, description)
  {};

  void execute_skill_on_franka(run_loop* run_loop,
                               FrankaRobot* robot,
                               FrankaGripper* gripper,
                               RobotStateData* robot_state_data) override;

 private:
  bool return_status_{false};
};

#endif  // FRANKA_INTERFACE_SKILLS_JOINT_VELOCITY_SKILL_H_