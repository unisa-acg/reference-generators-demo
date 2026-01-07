/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   math_utilities.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    sep 22, 2025
 *
 * Refer to the header file for current description of this module.
 *
 * -------------------------------------------------------------------
 */

#include "xxx_common_libraries/math_utilities.hpp"

namespace xxx_math_utilities
{

bool saturate_rate(const Eigen::VectorXd& last_vector, Eigen::VectorXd& vector, const std::vector<double>& torque_rate_limits)
{
  if (torque_rate_limits.empty() || static_cast<Eigen::Index>(torque_rate_limits.size()) != vector.size() || last_vector.size() != vector.size())
  {
    return false;
  }

  bool all_true{ true };
  for (std::size_t i = 0; i < torque_rate_limits.size(); i++)
  {
    const double rate_limit = torque_rate_limits[i];

    if (rate_limit < 0.0)
    {
      all_true = false;
      continue;
    }

    if (rate_limit > 0.0)
    {
      if (vector[i] - last_vector[i] > rate_limit)
      {
        vector[i] = last_vector[i] + rate_limit;
      }
      else if (vector[i] - last_vector[i] < -rate_limit)
      {
        vector[i] = last_vector[i] - rate_limit;
      }
    }
  }
  return all_true;
}

}  // namespace xxx_math_utilities
