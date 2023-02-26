#ifndef OB1_ARM_CONTROL__VISIBILITY_CONTROL_H_
#define OB1_ARM_CONTROL__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define OB1_ARM_CONTROL_EXPORT __attribute__ ((dllexport))
    #define OB1_ARM_CONTROL_IMPORT __attribute__ ((dllimport))
  #else
    #define OB1_ARM_CONTROL_EXPORT __declspec(dllexport)
    #define OB1_ARM_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef OB1_ARM_CONTROL_BUILDING_LIBRARY
    #define OB1_ARM_CONTROL_PUBLIC OB1_ARM_CONTROL_EXPORT
  #else
    #define OB1_ARM_CONTROL_PUBLIC OB1_ARM_CONTROL_IMPORT
  #endif
  #define OB1_ARM_CONTROL_PUBLIC_TYPE OB1_ARM_CONTROL_PUBLIC
  #define OB1_ARM_CONTROL_LOCAL
#else
  #define OB1_ARM_CONTROL_EXPORT __attribute__ ((visibility("default")))
  #define OB1_ARM_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define OB1_ARM_CONTROL_PUBLIC __attribute__ ((visibility("default")))
    #define OB1_ARM_CONTROL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define OB1_ARM_CONTROL_PUBLIC
    #define OB1_ARM_CONTROL_LOCAL
  #endif
  #define OB1_ARM_CONTROL_PUBLIC_TYPE
#endif
#endif  // OB1_ARM_CONTROL__VISIBILITY_CONTROL_H_
// Generated 08-Dec-2022 23:28:30
// Copyright 2019-2020 The MathWorks, Inc.
