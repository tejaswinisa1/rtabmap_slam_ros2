

#ifndef RTABMAP_SYNC__VISIBILITY_CONTROL_H_
#define RTABMAP_SYNC__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif


#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RTABMAP_SYNC_EXPORT __attribute__ ((dllexport))
    #define RTABMAP_SYNC_IMPORT __attribute__ ((dllimport))
  #else
    #define RTABMAP_SYNC_EXPORT __declspec(dllexport)
    #define RTABMAP_SYNC_IMPORT __declspec(dllimport)
  #endif
  #ifdef RTABMAP_SYNC_BUILDING_DLL
    #define RTABMAP_SYNC_PUBLIC RTABMAP_SYNC_EXPORT
  #else
    #define RTABMAP_SYNC_PUBLIC RTABMAP_SYNC_IMPORT
  #endif
  #define RTABMAP_SYNC_PUBLIC_TYPE RTABMAP_SYNC_PUBLIC
  #define RTABMAP_SYNC_LOCAL
#else
  #define RTABMAP_SYNC_EXPORT __attribute__ ((visibility("default")))
  #define RTABMAP_SYNC_IMPORT
  #if __GNUC__ >= 4
    #define RTABMAP_SYNC_PUBLIC __attribute__ ((visibility("default")))
    #define RTABMAP_SYNC_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RTABMAP_SYNC_PUBLIC
    #define RTABMAP_SYNC_LOCAL
  #endif
  #define RTABMAP_SYNC_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // RTABMAP_SYNC__VISIBILITY_CONTROL_H_

