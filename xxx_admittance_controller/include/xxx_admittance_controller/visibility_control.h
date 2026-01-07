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

#ifndef xxx_ADMITTANCE_CONTROLLER__VISIBILITY_CONTROL_H_
#define xxx_ADMITTANCE_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define xxx_ADMITTANCE_CONTROLLER_EXPORT __attribute__((dllexport))
#define xxx_ADMITTANCE_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define xxx_ADMITTANCE_CONTROLLER_EXPORT __declspec(dllexport)
#define xxx_ADMITTANCE_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef xxx_ADMITTANCE_CONTROLLER_BUILDING_DLL
#define xxx_ADMITTANCE_CONTROLLER_PUBLIC xxx_ADMITTANCE_CONTROLLER_EXPORT
#else
#define xxx_ADMITTANCE_CONTROLLER_PUBLIC xxx_ADMITTANCE_CONTROLLER_IMPORT
#endif
#define xxx_ADMITTANCE_CONTROLLER_PUBLIC_TYPE xxx_ADMITTANCE_CONTROLLER_PUBLIC
#define xxx_ADMITTANCE_CONTROLLER_LOCAL
#else
#define xxx_ADMITTANCE_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define xxx_ADMITTANCE_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define xxx_ADMITTANCE_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define xxx_ADMITTANCE_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define xxx_ADMITTANCE_CONTROLLER_PUBLIC
#define xxx_ADMITTANCE_CONTROLLER_LOCAL
#endif
#define xxx_ADMITTANCE_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // xxx_ADMITTANCE_CONTROLLER__VISIBILITY_CONTROL_H_
