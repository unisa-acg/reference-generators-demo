/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   visibility_control.h
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Feb 04, 2025
 *
 * This module contains the visibility control macros for the
 * motion_based_interaction_controller package.
 * -------------------------------------------------------------------
 */

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef ACG_ADMITTANCE_CONTROLLER__VISIBILITY_CONTROL_H_
#define ACG_ADMITTANCE_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ACG_ADMITTANCE_CONTROLLER_EXPORT __attribute__((dllexport))
#define ACG_ADMITTANCE_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define ACG_ADMITTANCE_CONTROLLER_EXPORT __declspec(dllexport)
#define ACG_ADMITTANCE_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef ACG_ADMITTANCE_CONTROLLER_BUILDING_DLL
#define ACG_ADMITTANCE_CONTROLLER_PUBLIC ACG_ADMITTANCE_CONTROLLER_EXPORT
#else
#define ACG_ADMITTANCE_CONTROLLER_PUBLIC ACG_ADMITTANCE_CONTROLLER_IMPORT
#endif
#define ACG_ADMITTANCE_CONTROLLER_PUBLIC_TYPE ACG_ADMITTANCE_CONTROLLER_PUBLIC
#define ACG_ADMITTANCE_CONTROLLER_LOCAL
#else
#define ACG_ADMITTANCE_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define ACG_ADMITTANCE_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define ACG_ADMITTANCE_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define ACG_ADMITTANCE_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define ACG_ADMITTANCE_CONTROLLER_PUBLIC
#define ACG_ADMITTANCE_CONTROLLER_LOCAL
#endif
#define ACG_ADMITTANCE_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // ACG_ADMITTANCE_CONTROLLER__VISIBILITY_CONTROL_H_
