.

#ifndef RTABMAP_RVIZ_PLUGINS__VISIBILITY_CONTROL_H_
#define RTABMAP_RVIZ_PLUGINS__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RTABMAP_RVIZ_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define RTABMAP_RVIZ_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define RTABMAP_RVIZ_PLUGINS_EXPORT __declspec(dllexport)
    #define RTABMAP_RVIZ_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef RTABMAP_RVIZ_PLUGINS_BUILDING_DLL
    #define RTABMAP_RVIZ_PLUGINS_PUBLIC RTABMAP_RVIZ_PLUGINS_EXPORT
  #else
    #define RTABMAP_RVIZ_PLUGINS_PUBLIC RTABMAP_RVIZ_PLUGINS_IMPORT
  #endif
  #define RTABMAP_RVIZ_PLUGINS_PUBLIC_TYPE RTABMAP_RVIZ_PLUGINS_PUBLIC
  #define RTABMAP_RVIZ_PLUGINS_LOCAL
#else
  #define RTABMAP_RVIZ_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define RTABMAP_RVIZ_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define RTABMAP_RVIZ_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define RTABMAP_RVIZ_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RTABMAP_RVIZ_PLUGINS_PUBLIC
    #define RTABMAP_RVIZ_PLUGINS_LOCAL
  #endif
  #define RTABMAP_RVIZ_PLUGINS_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // RTABMAP_RVIZ_PLUGINS__VISIBILITY_CONTROL_H_
