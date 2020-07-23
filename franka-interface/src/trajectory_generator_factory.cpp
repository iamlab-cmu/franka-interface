//
// Created by mohit on 12/18/18.
//

#include "franka-interface/trajectory_generator_factory.h"

#include <iostream>
#include <franka-interface-common/definitions.h>

#include "franka-interface/skills/base_meta_skill.h"
#include "franka-interface/skills/base_skill.h"
#include "franka-interface/trajectory_generator/cubic_hermite_spline_joint_trajectory_generator.h"
#include "franka-interface/trajectory_generator/cubic_hermite_spline_pose_trajectory_generator.h"
#include "franka-interface/trajectory_generator/goal_pose_dmp_trajectory_generator.h"
#include "franka-interface/trajectory_generator/gripper_trajectory_generator.h"
#include "franka-interface/trajectory_generator/impulse_trajectory_generator.h"
#include "franka-interface/trajectory_generator/joint_dmp_trajectory_generator.h"
#include "franka-interface/trajectory_generator/linear_force_position_trajectory_generator.h"
#include "franka-interface/trajectory_generator/linear_pose_trajectory_generator.h"
#include "franka-interface/trajectory_generator/linear_joint_trajectory_generator.h"
#include "franka-interface/trajectory_generator/min_jerk_joint_trajectory_generator.h"
#include "franka-interface/trajectory_generator/min_jerk_pose_trajectory_generator.h"
#include "franka-interface/trajectory_generator/pass_through_force_position_trajectory_generator.h"
#include "franka-interface/trajectory_generator/pass_through_joint_trajectory_generator.h"
#include "franka-interface/trajectory_generator/pass_through_pose_trajectory_generator.h"
#include "franka-interface/trajectory_generator/pose_dmp_trajectory_generator.h"
#include "franka-interface/trajectory_generator/relative_linear_pose_trajectory_generator.h"
#include "franka-interface/trajectory_generator/relative_min_jerk_pose_trajectory_generator.h"
#include "franka-interface/trajectory_generator/sine_joint_trajectory_generator.h"
#include "franka-interface/trajectory_generator/sine_pose_trajectory_generator.h"
#include "franka-interface/trajectory_generator/stay_in_initial_joints_trajectory_generator.h"
#include "franka-interface/trajectory_generator/stay_in_initial_pose_trajectory_generator.h"

TrajectoryGenerator* TrajectoryGeneratorFactory::getTrajectoryGeneratorForSkill(
    SharedBufferTypePtr buffer, SensorDataManager* sensor_data_manager) {
  TrajectoryGeneratorType trajectory_generator_type = static_cast<TrajectoryGeneratorType>(buffer[0]);
  TrajectoryGenerator *trajectory_generator = nullptr;

  std::string trajectory_generator_type_name;

  switch (trajectory_generator_type) {
    case TrajectoryGeneratorType::CubicHermiteSplineJointTrajectoryGenerator:
      trajectory_generator_type_name = "CubicHermiteSplineJointTrajectoryGenerator";
      trajectory_generator = new CubicHermiteSplineJointTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::CubicHermiteSplinePoseTrajectoryGenerator:
      trajectory_generator_type_name = "CubicHermiteSplinePoseTrajectoryGenerator";
      trajectory_generator = new CubicHermiteSplinePoseTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::GoalPoseDmpTrajectoryGenerator:
      trajectory_generator_type_name = "GoalPoseDmpTrajectoryGenerator";
      trajectory_generator = new GoalPoseDmpTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::GripperTrajectoryGenerator:
      trajectory_generator_type_name = "GripperTrajectoryGenerator";
      trajectory_generator = new GripperTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::ImpulseTrajectoryGenerator:
      trajectory_generator_type_name = "ImpulseTrajectoryGenerator";
      trajectory_generator = new ImpulseTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::JointDmpTrajectoryGenerator:
      trajectory_generator_type_name = "JointDmpTrajectoryGenerator";
      trajectory_generator = new JointDmpTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::LinearForcePositionTrajectoryGenerator:
      trajectory_generator_type_name = "LinearForcePositionTrajectoryGenerator";
      trajectory_generator = new LinearForcePositionTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::LinearJointTrajectoryGenerator:
      trajectory_generator_type_name = "LinearJointTrajectoryGenerator";
      trajectory_generator = new LinearJointTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::LinearPoseTrajectoryGenerator:
      trajectory_generator_type_name = "LinearPoseTrajectoryGenerator";
      trajectory_generator = new LinearPoseTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::MinJerkJointTrajectoryGenerator:
      trajectory_generator_type_name = "MinJerkJointTrajectoryGenerator";
      trajectory_generator = new MinJerkJointTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::MinJerkPoseTrajectoryGenerator:
      trajectory_generator_type_name = "MinJerkPoseTrajectoryGenerator";
      trajectory_generator = new MinJerkPoseTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::PassThroughForcePositionTrajectoryGenerator:
      trajectory_generator_type_name = "PassThroughForcePositionTrajectoryGenerator";
      trajectory_generator = new PassThroughForcePositionTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::PassThroughJointTrajectoryGenerator:
      trajectory_generator_type_name = "PassThroughJointTrajectoryGenerator";
      trajectory_generator = new PassThroughJointTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::PassThroughPoseTrajectoryGenerator:
      trajectory_generator_type_name = "PassThroughPoseTrajectoryGenerator";
      trajectory_generator = new PassThroughPoseTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::PoseDmpTrajectoryGenerator:
      trajectory_generator_type_name = "PoseDmpTrajectoryGenerator";
      trajectory_generator = new PoseDmpTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::RelativeLinearPoseTrajectoryGenerator:
      trajectory_generator_type_name = "RelativeLinearPoseTrajectoryGenerator";
      trajectory_generator = new RelativeLinearPoseTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::RelativeMinJerkPoseTrajectoryGenerator:
      trajectory_generator_type_name = "RelativeMinJerkPoseTrajectoryGenerator";
      trajectory_generator = new RelativeMinJerkPoseTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::SineJointTrajectoryGenerator:
      trajectory_generator_type_name = "SineJointTrajectoryGenerator";
      trajectory_generator = new SineJointTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::SinePoseTrajectoryGenerator:
      trajectory_generator_type_name = "SinePoseTrajectoryGenerator";
      trajectory_generator = new SinePoseTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::StayInInitialJointsTrajectoryGenerator:
      trajectory_generator_type_name = "StayInInitialJointsTrajectoryGenerator";
      trajectory_generator = new StayInInitialJointsTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    case TrajectoryGeneratorType::StayInInitialPoseTrajectoryGenerator:
      trajectory_generator_type_name = "StayInInitialPoseTrajectoryGenerator";
      trajectory_generator = new StayInInitialPoseTrajectoryGenerator(buffer, sensor_data_manager);
      break;
    default:
      std::cout << "Cannot create Trajectory Generator with type:" << 
      static_cast<std::underlying_type<TrajectoryGeneratorType>::type>(trajectory_generator_type) << 
      "\n";
      return nullptr;
  }
  std::cout << "Trajectory Generator Type: " << trajectory_generator_type_name << std::endl;

  trajectory_generator->parse_parameters();
  return trajectory_generator;
}

