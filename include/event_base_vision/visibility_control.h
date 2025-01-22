#ifndef EVENT_BASE_VISION__VISIBILITY_CONTROL_H_
#define EVENT_BASE_VISION__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define EVENT_BASE_VISION_EXPORT __attribute__ ((dllexport))
    #define EVENT_BASE_VISION_IMPORT __attribute__ ((dllimport))
  #else
    #define EVENT_BASE_VISION_EXPORT __declspec(dllexport)
    #define EVENT_BASE_VISION_IMPORT __declspec(dllimport)
  #endif
  #ifdef EVENT_BASE_VISION_BUILDING_LIBRARY
    #define EVENT_BASE_VISION_PUBLIC EVENT_BASE_VISION_EXPORT
  #else
    #define EVENT_BASE_VISION_PUBLIC EVENT_BASE_VISION_IMPORT
  #endif
  #define EVENT_BASE_VISION_PUBLIC_TYPE EVENT_BASE_VISION_PUBLIC
  #define EVENT_BASE_VISION_LOCAL
#else
  #define EVENT_BASE_VISION_EXPORT __attribute__ ((visibility("default")))
  #define EVENT_BASE_VISION_IMPORT
  #if __GNUC__ >= 4
    #define EVENT_BASE_VISION_PUBLIC __attribute__ ((visibility("default")))
    #define EVENT_BASE_VISION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define EVENT_BASE_VISION_PUBLIC
    #define EVENT_BASE_VISION_LOCAL
  #endif
  #define EVENT_BASE_VISION_PUBLIC_TYPE
#endif

#endif  // EVENT_BASE_VISION__VISIBILITY_CONTROL_H_