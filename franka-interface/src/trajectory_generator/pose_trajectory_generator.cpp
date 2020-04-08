#include "franka-interface/trajectory_generator/pose_trajectory_generator.h"

void PoseTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = pose_trajectory_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    run_time_ = pose_trajectory_params_.run_time();

    if(pose_trajectory_params_.pose_size() == 16){
      for(size_t i = 0; i < goal_pose_.size(); i++) {
        goal_pose_[i] = pose_trajectory_params_.pose(i);
      }

      Eigen::Affine3d goal_transform(Eigen::Matrix4d::Map(goal_pose_.data()));
      goal_position_ = Eigen::Vector3d(goal_transform.translation());
      goal_orientation_ = Eigen::Quaterniond(goal_transform.linear());
    } else {

      for(int i = 0; i < 3; i++) {
        goal_position_[i] = pose_trajectory_params_.position(i);
      }

      goal_orientation_.w() = pose_trajectory_params_.quaternion(0);
      goal_orientation_.x() = pose_trajectory_params_.quaternion(1);
      goal_orientation_.y() = pose_trajectory_params_.quaternion(2);
      goal_orientation_.z() = pose_trajectory_params_.quaternion(3);
    }
  } else {
    std::cout << "Parsing PoseTrajectoryGenerator params failed. Data size = " << data_size << std::endl;
  }
}

void PoseTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state,
                                                    SkillType skill_type) {
  initialize_initial_and_desired_poses(robot_state, skill_type);
  fix_goal_quaternion();
}

void PoseTrajectoryGenerator::initialize_initial_and_desired_poses(const franka::RobotState &robot_state,
                                                                   SkillType skill_type) {
  switch(skill_type) {
    case SkillType::ImpedanceControlSkill:
      // Use O_T_EE as the initial pose for Impedance Control for safety reasons
      initial_pose_ = robot_state.O_T_EE;
      desired_pose_ = robot_state.O_T_EE;
      break;
    case SkillType::CartesianPoseSkill:
      // Use O_T_EE_c as the initial pose for Cartesian Pose Control to 
      // avoid trajectory discontinuity errors
      initial_pose_ = robot_state.O_T_EE_c;
      desired_pose_ = robot_state.O_T_EE_c;
      break;
    default:
      // Default to using O_T_EE as the initial pose for safety reasons 
      initial_pose_ = robot_state.O_T_EE;
      desired_pose_ = robot_state.O_T_EE;
  }

  initial_transform_ = Eigen::Affine3d(Eigen::Matrix4d::Map(initial_pose_.data()));
  initial_position_ = Eigen::Vector3d(initial_transform_.translation());
  initial_orientation_ = Eigen::Quaterniond(initial_transform_.linear());
  desired_position_ = Eigen::Vector3d(initial_transform_.translation());
  desired_orientation_ = Eigen::Quaterniond(initial_transform_.linear());
}

void PoseTrajectoryGenerator::fix_goal_quaternion(){
  // Flip the goal quaternion if the initial orientation dotted with the goal
  // orientation is negative.

  initial_orientation_.normalize();
  goal_orientation_.normalize();

  double quaternion_dot_product = initial_orientation_.coeffs().dot(goal_orientation_.coeffs());
  if (quaternion_dot_product < 0.0) {
    goal_orientation_.coeffs() << -goal_orientation_.coeffs();
  }

  same_orientation = abs(quaternion_dot_product) > quaternion_dist_threshold;
}

void PoseTrajectoryGenerator::calculate_desired_pose() {
  if (same_orientation) {
    calculate_desired_position();
  } else {
    Eigen::Affine3d desired_pose_affine = Eigen::Affine3d::Identity();
    desired_pose_affine.translate(desired_position_);
    // Normalize desired orientation quaternion to avoid precision issues
    desired_orientation_.normalize();
    desired_pose_affine.rotate(desired_orientation_);
    Eigen::Matrix4d desired_pose_matrix = desired_pose_affine.matrix();

    for(int i = 0; i < 4; i++) {
      for(int j = 0; j < 4; j++) {
        desired_pose_[4*i+j] = desired_pose_matrix(j,i); // Column wise
      }
    }
  }
}

void PoseTrajectoryGenerator::calculate_desired_position() {
  // Just change the desired position and not the orientation.
  for (int i = 0; i < 3; i++) {
    desired_pose_[12 + i] = desired_position_(i);
  }
}

const std::array<double, 16>& PoseTrajectoryGenerator::get_desired_pose() const {
  return desired_pose_;
}

const Eigen::Vector3d& PoseTrajectoryGenerator::get_desired_position() const {
  return desired_position_;
}

const Eigen::Quaterniond& PoseTrajectoryGenerator::get_desired_orientation() const {
  return desired_orientation_;
}

const Eigen::Vector3d& PoseTrajectoryGenerator::get_goal_position() const {
  return goal_position_;
}

const Eigen::Quaterniond& PoseTrajectoryGenerator::get_goal_orientation() const {
  return goal_orientation_;
}