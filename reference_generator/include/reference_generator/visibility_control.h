/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   visibility_control.h
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Dec 03, 2024
 *
 * This module contains the visibility control macros for the
 * reference_generator package.
 * -------------------------------------------------------------------
 */

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef REFERENCE_GENERATOR__VISIBILITY_CONTROL_H_
#define REFERENCE_GENERATOR__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define REFERENCE_GENERATOR_EXPORT __attribute__((dllexport))
#define REFERENCE_GENERATOR_IMPORT __attribute__((dllimport))
#else
#define REFERENCE_GENERATOR_EXPORT __declspec(dllexport)
#define REFERENCE_GENERATOR_IMPORT __declspec(dllimport)
#endif
#ifdef REFERENCE_GENERATOR_BUILDING_DLL
#define REFERENCE_GENERATOR_PUBLIC REFERENCE_GENERATOR_EXPORT
#else
#define REFERENCE_GENERATOR_PUBLIC REFERENCE_GENERATOR_IMPORT
#endif
#define REFERENCE_GENERATOR_PUBLIC_TYPE REFERENCE_GENERATOR_PUBLIC
#define REFERENCE_GENERATOR_LOCAL
#else
#define REFERENCE_GENERATOR_EXPORT __attribute__((visibility("default")))
#define REFERENCE_GENERATOR_IMPORT
#if __GNUC__ >= 4
#define REFERENCE_GENERATOR_PUBLIC __attribute__((visibility("default")))
#define REFERENCE_GENERATOR_LOCAL __attribute__((visibility("hidden")))
#else
#define REFERENCE_GENERATOR_PUBLIC
#define REFERENCE_GENERATOR_LOCAL
#endif
#define REFERENCE_GENERATOR_PUBLIC_TYPE
#endif

#endif  // REFERENCE_GENERATOR__VISIBILITY_CONTROL_H_
