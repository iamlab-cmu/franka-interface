//
// Created by mohit on 12/9/18.
//

#include "franka-interface/skills/base_meta_skill.h"

#include "franka-interface/run_loop.h"

int BaseMetaSkill::getMetaSkillId() {
  return skill_idx_;
}

bool BaseMetaSkill::isComposableSkill() {
  return is_composable_;
}

void BaseMetaSkill::setMetaSkillStatus(SkillStatus new_status) {
  skill_status_ = new_status;
}

SkillStatus BaseMetaSkill::getCurrentMetaSkillStatus() {
  return skill_status_;
}

void BaseMetaSkill::execute_skill_on_franka(run_loop* run_loop, FrankaRobot *robot,
                                            RobotStateData *robot_state_data) {
  BaseSkill* skill = run_loop->getSkillInfoManager()->get_current_skill();
  if (skill != nullptr) {
    model_ = robot->getModel();
    skill->execute_skill_on_franka(run_loop, robot, robot_state_data);
    run_loop->finish_current_skill(skill);
  }
}
