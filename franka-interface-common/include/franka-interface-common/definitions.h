#ifndef FRANKA_INTERFACE_COMMON_DEFINITIONS_H_
#define FRANKA_INTERFACE_COMMON_DEFINITIONS_H_

#include <stdint.h>

/*
 *
 *  Important: Any Changes here should also be reflected in changes
 *  in the frankpy franka_interface_common_definitions.py file as well.
 *
 *  The order of the enums matter!!
 * 
 */

// SharedBuffer type to share memory (Change size later)
typedef uint8_t SharedBufferType;
typedef SharedBufferType* SharedBufferTypePtr;

typedef uint8_t SensorBufferType;
typedef SensorBufferType* SensorBufferTypePtr;

// Enum for Skill Types
enum class SkillType : uint8_t {
    CartesianPoseSkill,
    ForceTorqueSkill,
    GripperSkill,
    ImpedanceControlSkill,
    JointPositionSkill,
};

// Enum for Meta Skill Types
enum class MetaSkillType : uint8_t {
    BaseMetaSkill,
    JointPositionContinuousSkill,
};

// Enum for Trajectory Generator Types
enum class TrajectoryGeneratorType : uint8_t {
    CubicHermiteSplineJointTrajectoryGenerator,
    CubicHermiteSplinePoseTrajectoryGenerator,
    GoalPoseDmpTrajectoryGenerator,
    GripperTrajectoryGenerator,
    ImpulseTrajectoryGenerator,
    JointDmpTrajectoryGenerator,
    LinearForcePositionTrajectoryGenerator,
    LinearJointTrajectoryGenerator,
    LinearPoseTrajectoryGenerator,
    MinJerkJointTrajectoryGenerator,
    MinJerkPoseTrajectoryGenerator,
    PassThroughForcePositionTrajectoryGenerator,
    PassThroughJointTrajectoryGenerator,
    PassThroughPoseTrajectoryGenerator,
    PoseDmpTrajectoryGenerator,
    RelativeLinearPoseTrajectoryGenerator,
    RelativeMinJerkPoseTrajectoryGenerator,
    SineJointTrajectoryGenerator,
    SinePoseTrajectoryGenerator,
    StayInInitialJointsTrajectoryGenerator,
    StayInInitialPoseTrajectoryGenerator,
};

// Enum for Feedback Controller Types
enum class FeedbackControllerType : uint8_t {
    CartesianImpedanceFeedbackController,
    EECartesianImpedanceFeedbackController,
    ForceAxisImpedenceFeedbackController,
    ForcePositionFeedbackController,
    JointImpedanceFeedbackController,
    NoopFeedbackController,
    PassThroughFeedbackController,
    SetInternalImpedanceFeedbackController,
};

// Enum for Termination Handler Types
enum class TerminationHandlerType : uint8_t {
    ContactTerminationHandler,
    FinalJointTerminationHandler,
    FinalPoseTerminationHandler,
    NoopTerminationHandler,
    TimeTerminationHandler,
};

// Enum for Skill Statuses
enum class SkillStatus : uint8_t { 
  TO_START,
  RUNNING,
  FINISHED,
  VIRT_COLL_ERR,
}; 

enum class SensorDataManagerReadStatus : uint8_t {
  FAIL_TO_GET_LOCK,
  FAIL_TO_READ,
  NO_NEW_MESSAGE,
  SUCCESS,
};

enum class SensorDataMessageType : uint8_t {
  BOUNDING_BOX,
  CARTESIAN_IMPEDANCE,
  FORCE_POSITION,
  FORCE_POSITION_GAINS,
  JOINT_POSITION_VELOCITY,
  JOINT_POSITION,
  POSE_POSITION_VELOCITY,
  POSE_POSITION,
  SHOULD_TERMINATE,
};

#endif  // FRANKA_INTERFACE_COMMON_DEFINITIONS_H_
