/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   interpolation.hpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 7, 2025
 *
 * This library provides functions for interpolation.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <tf2_eigen/tf2_eigen.hpp>

#include <acg_control_msgs/msg/joint_wrench_point.hpp>
#include <acg_control_msgs/msg/joint_trajectory_point.hpp>
#include <acg_control_msgs/msg/task_space_point.hpp>
#include <acg_control_msgs/msg/task_space_trajectory_point.hpp>

namespace acg_interpolation
{

/**
 * @brief Performs element-wise linear interpolation between two Eigen matrices.
 *
 * This function computes the linear interpolation between two Eigen matrices at a given normalized time \p t_normalized.
 * If the matrices dimensions are defined at compile time, the coherence of the dimensions is checked at compile time.
 * If the matrices dimensions are defined at runtime, the coherence of the dimensions is not checked.
 *
 * @param[in] first_point The first point.
 * @param[in] second_point The second point.
 * @param[in] t_normalized The normalized time in the range [0, 1] at which to interpolate between the two points.
 * @param[out] interpolated_point The interpolated point.
 */
template <typename Derived>
void linearly_interpolate(const Eigen::MatrixBase<Derived>& first_point, const Eigen::MatrixBase<Derived>& second_point, const double t_normalized,
                          Eigen::MatrixBase<Derived>& interpolated_point)
{
  // Computes the element-wise linear interpolation between two Eigen matrices.
  interpolated_point = first_point + (second_point - first_point) * t_normalized;
}

/**
 * @brief Performs element-wise linear interpolation between two vector of doubles.
 *
 * This function computes the linear interpolation between two vector of doubles at a given normalized time \p t_normalized.
 * The coherence of the dimensions of input and output vectors is checked at runtime.
 *
 * @param[in] first_point The first point.
 * @param[in] second_point The second point.
 * @param[in] t_normalized The normalized time in the range [0, 1] at which to interpolate between the two points.
 * @param[out] interpolated_point The interpolated point.
 */
void linearly_interpolate(const std::vector<double>& first_point, const std::vector<double>& second_point, const double t_normalized,
                          std::vector<double>& interpolated_point);

/**
 * @brief Sample a joint trajectory point using linear interpolation.
 *
 * This function calculates a joint trajectory point by performing linear interpolation between the first and second points, and returns the result at
 * time \p t, storing the interpolated point in the \p interpolated_point variable.
 *
 * @throws \c std::runtime_error if the time \p t is out of bounds, if the first timestamp is greater than the second timestamp or if the points have
 * different sizes.
 *
 * @param[in] first_point The initial joint trajectory point.
 * @param[in] second_point The subsequent joint trajectory point.
 * @param[in] t The time at which to sample the joint trajectory point.
 * @param[out] interpolated_point The resulting interpolated joint trajectory point.
 */
void linearly_interpolate(const acg_control_msgs::msg::JointTrajectoryPoint& first_point,
                          const acg_control_msgs::msg::JointTrajectoryPoint& second_point, const double t,
                          acg_control_msgs::msg::JointWrenchPoint& interpolated_point);

/**
 * @brief Sample a task space trajectory point using a linear interpolation.
 *
 * This function samples a task space trajectory point by performing linear interpolation between two task space trajectory points at a given time
 * \p t. The result is stored in the \p interpolated_point variable.
 *
 * @throws \c std::runtime_error if the time \p t is out of bounds or if the first timestamp is greater than the second timestamp.
 *
 * @param[in] first_point The first task space trajectory point.
 * @param[in] second_point The second task space trajectory point.
 * @param[in] t The time at which to sample the task space trajectory point.
 * @param[out] interpolated_point The interpolated task space point.
 */
void linearly_interpolate(const acg_control_msgs::msg::TaskSpaceTrajectoryPoint& first_point,
                          const acg_control_msgs::msg::TaskSpaceTrajectoryPoint& second_point, const double t,
                          acg_control_msgs::msg::TaskSpacePoint& interpolated_point);

}  // namespace acg_interpolation
