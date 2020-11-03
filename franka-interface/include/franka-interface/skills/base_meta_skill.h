#ifndef FRANKA_INTERFACE_SKILLS_BASE_META_SKILL_H_
#define FRANKA_INTERFACE_SKILLS_BASE_META_SKILL_H_

#include "franka-interface/skills/base_skill.h"

class run_loop;

class BaseMetaSkill {
 public:
  BaseMetaSkill(int skill_idx): skill_idx_(skill_idx),
                                skill_status_(SkillStatus::TO_START) 
  {};

  int getMetaSkillId();

  virtual bool isComposableSkill();

  void setMetaSkillStatus(SkillStatus new_task_status);

  SkillStatus getCurrentMetaSkillStatus();

  virtual void execute_skill_on_franka(run_loop* run_loop, 
                                       FrankaRobot* robot, 
                                       FrankaGripper* gripper,
                                       RobotStateData* robot_state_data);

 protected:
  int skill_idx_;
  SkillStatus skill_status_;
  bool is_composable_{false};
  double current_period_;
  franka::Model* model_= nullptr;
};

#endif  // FRANKA_INTERFACE_SKILLS_BASE_META_SKILL_H_