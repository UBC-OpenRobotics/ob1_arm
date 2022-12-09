#ifndef RELAXED_IK__VISIBILITY_CONTROL_H_
#define RELAXED_IK__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RELAXED_IK_EXPORT __attribute__ ((dllexport))
    #define RELAXED_IK_IMPORT __attribute__ ((dllimport))
  #else
    #define RELAXED_IK_EXPORT __declspec(dllexport)
    #define RELAXED_IK_IMPORT __declspec(dllimport)
  #endif
  #ifdef RELAXED_IK_BUILDING_LIBRARY
    #define RELAXED_IK_PUBLIC RELAXED_IK_EXPORT
  #else
    #define RELAXED_IK_PUBLIC RELAXED_IK_IMPORT
  #endif
  #define RELAXED_IK_PUBLIC_TYPE RELAXED_IK_PUBLIC
  #define RELAXED_IK_LOCAL
#else
  #define RELAXED_IK_EXPORT __attribute__ ((visibility("default")))
  #define RELAXED_IK_IMPORT
  #if __GNUC__ >= 4
    #define RELAXED_IK_PUBLIC __attribute__ ((visibility("default")))
    #define RELAXED_IK_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RELAXED_IK_PUBLIC
    #define RELAXED_IK_LOCAL
  #endif
  #define RELAXED_IK_PUBLIC_TYPE
#endif
#endif  // RELAXED_IK__VISIBILITY_CONTROL_H_
// Generated 08-Dec-2022 23:28:31
// Copyright 2019-2020 The MathWorks, Inc.
