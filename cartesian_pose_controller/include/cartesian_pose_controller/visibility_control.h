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
 * -------------------------------------------------------------------
 */

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef CARTESIAN_POSE_CONTROLLER__VISIBILITY_CONTROL_H_
#define CARTESIAN_POSE_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CARTESIAN_POSE_CONTROLLER_EXPORT __attribute__((dllexport))
#define CARTESIAN_POSE_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define CARTESIAN_POSE_CONTROLLER_EXPORT __declspec(dllexport)
#define CARTESIAN_POSE_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef CARTESIAN_POSE_CONTROLLER_BUILDING_DLL
#define CARTESIAN_POSE_CONTROLLER_PUBLIC CARTESIAN_POSE_CONTROLLER_EXPORT
#else
#define CARTESIAN_POSE_CONTROLLER_PUBLIC CARTESIAN_POSE_CONTROLLER_IMPORT
#endif
#define CARTESIAN_POSE_CONTROLLER_PUBLIC_TYPE CARTESIAN_POSE_CONTROLLER_PUBLIC
#define CARTESIAN_POSE_CONTROLLER_LOCAL
#else
#define CARTESIAN_POSE_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define CARTESIAN_POSE_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define CARTESIAN_POSE_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define CARTESIAN_POSE_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define CARTESIAN_POSE_CONTROLLER_PUBLIC
#define CARTESIAN_POSE_CONTROLLER_LOCAL
#endif
#define CARTESIAN_POSE_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // CARTESIAN_POSE_CONTROLLER__VISIBILITY_CONTROL_H_
