#ifndef OB1_ARM_HW_INTERFACE__VISIBILITY_CONTROL_H_
#define OB1_ARM_HW_INTERFACE__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define OB1_ARM_HW_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define OB1_ARM_HW_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define OB1_ARM_HW_INTERFACE_EXPORT __declspec(dllexport)
    #define OB1_ARM_HW_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef OB1_ARM_HW_INTERFACE_BUILDING_LIBRARY
    #define OB1_ARM_HW_INTERFACE_PUBLIC OB1_ARM_HW_INTERFACE_EXPORT
  #else
    #define OB1_ARM_HW_INTERFACE_PUBLIC OB1_ARM_HW_INTERFACE_IMPORT
  #endif
  #define OB1_ARM_HW_INTERFACE_PUBLIC_TYPE OB1_ARM_HW_INTERFACE_PUBLIC
  #define OB1_ARM_HW_INTERFACE_LOCAL
#else
  #define OB1_ARM_HW_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define OB1_ARM_HW_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define OB1_ARM_HW_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define OB1_ARM_HW_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define OB1_ARM_HW_INTERFACE_PUBLIC
    #define OB1_ARM_HW_INTERFACE_LOCAL
  #endif
  #define OB1_ARM_HW_INTERFACE_PUBLIC_TYPE
#endif
#endif  // OB1_ARM_HW_INTERFACE__VISIBILITY_CONTROL_H_
// Generated 08-Dec-2022 23:28:30
// Copyright 2019-2020 The MathWorks, Inc.
