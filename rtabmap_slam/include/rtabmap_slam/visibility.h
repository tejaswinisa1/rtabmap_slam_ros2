

#ifndef RTABMAP_SLAM__VISIBILITY_CONTROL_H_
#define RTABMAP_SLAM__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RTABMAP_SLAM_EXPORT __attribute__ ((dllexport))
    #define RTABMAP_SLAM_IMPORT __attribute__ ((dllimport))
  #else
    #define RTABMAP_SLAM_EXPORT __declspec(dllexport)
    #define RTABMAP_SLAM_IMPORT __declspec(dllimport)
  #endif
  #ifdef RTABMAP_SLAM_BUILDING_DLL
    #define RTABMAP_SLAM_PUBLIC RTABMAP_SLAM_EXPORT
  #else
    #define RTABMAP_SLAM_PUBLIC RTABMAP_SLAM_IMPORT
  #endif
  #define RTABMAP_SLAM_PUBLIC_TYPE RTABMAP_SLAM_PUBLIC
  #define RTABMAP_SLAM_LOCAL
#else
  #define RTABMAP_SLAM_EXPORT __attribute__ ((visibility("default")))
  #define RTABMAP_SLAM_IMPORT
  #if __GNUC__ >= 4
    #define RTABMAP_SLAM_PUBLIC __attribute__ ((visibility("default")))
    #define RTABMAP_SLAM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RTABMAP_SLAM_PUBLIC
    #define RTABMAP_SLAM_LOCAL
  #endif
  #define RTABMAP_SLAM_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // RTABMAP_SLAM__VISIBILITY_CONTROL_H_

