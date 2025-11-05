/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   visibility_control.h
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Apr 07, 2025
 *
 * This module contains the visibility control macros for the
 * cartesian_pose_controller package.
 *
 * -------------------------------------------------------------------
 */

#ifndef GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_CONTROL_H_
#define GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki: https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_EXPORT __attribute__((dllexport))
#define GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_IMPORT __attribute__((dllimport))
#else
#define GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_EXPORT __declspec(dllexport)
#define GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_IMPORT __declspec(dllimport)
#endif
#ifdef GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_BUILDING_DLL
#define GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_PUBLIC GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_EXPORT
#else
#define GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_PUBLIC GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_IMPORT
#endif
#define GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_PUBLIC_TYPE GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_PUBLIC
#define GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_LOCAL
#else
#define GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_EXPORT __attribute__((visibility("default")))
#define GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_IMPORT
#if __GNUC__ >= 4
#define GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_PUBLIC __attribute__((visibility("default")))
#define GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_LOCAL __attribute__((visibility("hidden")))
#else
#define GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_PUBLIC
#define GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_LOCAL
#endif
#define GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_PUBLIC_TYPE
#endif

#endif  // GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_CONTROL_H_
