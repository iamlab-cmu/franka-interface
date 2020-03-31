//
// Created by mohit on 11/20/18.
//

#include "franka-interface/skill_info_manager.h"

#include <cassert>

SkillInfoManager::SkillInfoManager() {
}

BaseSkill* SkillInfoManager::get_current_skill() {
  if (skill_list_.size() == 0) {
    // returns NULL
    return 0;
  }
  return skill_list_.back();
}

BaseMetaSkill* SkillInfoManager::get_current_meta_skill() {
  if (meta_skill_list_.size() == 0) {
    // returns NULL
    return 0;
  }
  return meta_skill_list_.back();
}

bool SkillInfoManager::is_currently_executing_skill() {
  if (skill_list_.size() == 0){
    return false;
  }
  BaseSkill*skill = skill_list_.back();
  SkillStatus status = skill->get_current_skill_status();
  return (status == SkillStatus::TO_START or status == SkillStatus::RUNNING);
}

bool SkillInfoManager::is_waiting_for_new_skill() {
  if (skill_list_.size() == 0){
    return true;
  }
  return (*skill_list_.back()).get_current_skill_status() == SkillStatus::FINISHED;
}

void SkillInfoManager::add_skill(BaseSkill *skill) {
  assert(is_waiting_for_new_skill());
  skill_list_.push_back(skill);
}

void SkillInfoManager::add_meta_skill(BaseMetaSkill *skill) {
  meta_skill_list_.push_back(skill);
}

BaseMetaSkill* SkillInfoManager::get_meta_skill_with_id(int meta_skill_id) {
  for (auto it = meta_skill_list_.rbegin(); it != meta_skill_list_.rend(); it++) {
    if ((*it)->getMetaSkillId() == meta_skill_id) {
      return *it;
    }
  }
  return nullptr;
}

void SkillInfoManager::clear_skill_and_meta_skill_list() {
  skill_list_.clear();
  meta_skill_list_.clear();
}
