#ifndef FRANKA_INTERFACE_SKILLS_JOINT_POSITION_DYNAMIC_INTERP_SKILL_H_
#define FRANKA_INTERFACE_SKILLS_JOINT_POSITION_DYNAMIC_INTERP_SKILL_H_

#include "franka-interface/skills/base_skill.h"

/**
 * This skill was used to demonstrate the use of sensor values. Basically, the idea is that the
 * skill receives some target joint position from the server when it is initialized. While
 * executing the trajectory to go to this target joint position, it receives a new joint position
 * from the sensor buffer. Once this skill receives the new target position it updates and tries
 * to move to this position.
 */
class JointPositionDynamicInterpSkill : public BaseSkill {
 public:
  JointPositionDynamicInterpSkill(int skill_idx, int meta_skill_idx, std::string description) :
      BaseSkill(skill_idx, meta_skill_idx, description)
  {};

  void execute_skill_on_franka(run_loop* run_loop,
                               FrankaRobot* robot,
                               RobotStateData* robot_state_data) override;

 private:
  bool return_status_{false};
};

#endif  // FRANKA_INTERFACE_SKILLS_JOINT_POSITION_DYNAMIC_INTERP_SKILL_H_