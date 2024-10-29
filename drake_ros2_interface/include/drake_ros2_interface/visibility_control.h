#ifndef DRAKE_ROS2_INTERFACE__VISIBILITY_CONTROL_H_
#define DRAKE_ROS2_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DRAKE_ROS2_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define DRAKE_ROS2_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define DRAKE_ROS2_INTERFACE_EXPORT __declspec(dllexport)
    #define DRAKE_ROS2_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef DRAKE_ROS2_INTERFACE_BUILDING_LIBRARY
    #define DRAKE_ROS2_INTERFACE_PUBLIC DRAKE_ROS2_INTERFACE_EXPORT
  #else
    #define DRAKE_ROS2_INTERFACE_PUBLIC DRAKE_ROS2_INTERFACE_IMPORT
  #endif
  #define DRAKE_ROS2_INTERFACE_PUBLIC_TYPE DRAKE_ROS2_INTERFACE_PUBLIC
  #define DRAKE_ROS2_INTERFACE_LOCAL
#else
  #define DRAKE_ROS2_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define DRAKE_ROS2_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define DRAKE_ROS2_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define DRAKE_ROS2_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DRAKE_ROS2_INTERFACE_PUBLIC
    #define DRAKE_ROS2_INTERFACE_LOCAL
  #endif
  #define DRAKE_ROS2_INTERFACE_PUBLIC_TYPE
#endif

endif  // DRAKE_ROS2_INTERFACE__VISIBILITY_CONTROL_H_
