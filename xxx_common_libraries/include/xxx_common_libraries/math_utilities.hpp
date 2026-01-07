/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   math_utilities.hpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    sep 22, 2025
 *
 * This library contains utility functions for mathematical
 * operations.
 *
 * -------------------------------------------------------------------
 */

#pragma once

// Eigen
#include <eigen3/Eigen/Core>

namespace xxx_math_utilities
{

/**
 * @brief Saturate the rate of change of a vector.
 *
 * This function limits the rate of change of each element in the vector
 * to a specified rate. If the rate of change exceeds the limit,
 * the value is clamped to stay within the allowed range.
 *
 * @param[in] last_vector The vector containing the last known values.
 * @param[in,out] vector The vector to be updated with the new values.
 * @param[in] torque_rate_limits The vector containing the maximum allowed rate of change for each element. Values of 0.0 indicate no limit for that
 * element, negative values are invalid.
 * @return True if the operation was successful, false otherwise.
 */
bool saturate_rate(const Eigen::VectorXd& last_vector, Eigen::VectorXd& vector, const std::vector<double>& torque_rate_limits);
}  // namespace xxx_math_utilities
