#ifndef FRANKA_ROS_INTERFACE__VISIBILITY_CONTROL_H_
#define FRANKA_ROS_INTERFACE__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FRANKA_ROS_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define FRANKA_ROS_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define FRANKA_ROS_INTERFACE_EXPORT __declspec(dllexport)
    #define FRANKA_ROS_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef FRANKA_ROS_INTERFACE_BUILDING_DLL
    #define FRANKA_ROS_INTERFACE_PUBLIC FRANKA_ROS_INTERFACE_EXPORT
  #else
    #define FRANKA_ROS_INTERFACE_PUBLIC FRANKA_ROS_INTERFACE_IMPORT
  #endif
  #define FRANKA_ROS_INTERFACE_PUBLIC_TYPE FRANKA_ROS_INTERFACE_PUBLIC
  #define FRANKA_ROS_INTERFACE_LOCAL
#else
  #define FRANKA_ROS_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define FRANKA_ROS_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define FRANKA_ROS_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define FRANKA_ROS_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FRANKA_ROS_INTERFACE_PUBLIC
    #define FRANKA_ROS_INTERFACE_LOCAL
  #endif
  #define FRANKA_ROS_INTERFACE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // FRANKA_ROS_INTERFACE__VISIBILITY_CONTROL_H_