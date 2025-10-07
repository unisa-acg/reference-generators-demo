/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   interpolation.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 7, 2025
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <rclcpp/rclcpp.hpp>

#include "acg_common_libraries/interpolation.hpp"
#include "acg_common_libraries/message_utilities.hpp"

namespace acg_interpolation
{

void linearly_interpolate(const std::vector<double>& first_point, const std::vector<double>& second_point, const double t_normalized,
                          std::vector<double>& interpolated_point)
{
  if (first_point.size() != second_point.size())
  {
    throw std::runtime_error("The input points must have the same size.");
  }

  if (interpolated_point.size() != first_point.size())
  {
    throw std::runtime_error("The output point must have the same size as the input points.");
  }

  for (std::size_t i = 0; i < first_point.size(); i++)
  {
    interpolated_point[i] = first_point[i] + (second_point[i] - first_point[i]) * t_normalized;
  }
}

void linearly_interpolate(const acg_control_msgs::msg::JointTrajectoryPoint& first_point,
                          const acg_control_msgs::msg::JointTrajectoryPoint& second_point, const double t,
                          acg_control_msgs::msg::JointWrenchPoint& interpolated_point)
{
  const double first_timestamp{ rclcpp::Duration(first_point.time_from_start).seconds() };
  const double second_timestamp{ rclcpp::Duration(second_point.time_from_start).seconds() };

  if (first_timestamp >= second_timestamp)
  {
    throw std::runtime_error("The first timestamp should be smaller than the second timestamp");
  }

  if (t < first_timestamp || t > second_timestamp)
  {
    throw std::runtime_error("Time t (" + std::to_string(t) + ") is out of bounds. It should be between " + std::to_string(first_timestamp) +
                             " and " + std::to_string(second_timestamp));
  }

  double t_normalized = (t - first_timestamp) / (second_timestamp - first_timestamp);

  // Interpolate the joint positions
  linearly_interpolate(first_point.point.positions, second_point.point.positions, t_normalized, interpolated_point.positions);

  // Interpolate the joint velocities
  linearly_interpolate(first_point.point.velocities, second_point.point.velocities, t_normalized, interpolated_point.velocities);

  // Interpolate the joint accelerations
  linearly_interpolate(first_point.point.accelerations, second_point.point.accelerations, t_normalized, interpolated_point.accelerations);

  // Interpolate the joint efforts
  linearly_interpolate(first_point.point.effort, second_point.point.effort, t_normalized, interpolated_point.effort);

  // Interpolate the wrench
  Eigen::Matrix<double, 6, 1> first_vector, second_vector, interpolated_vector;
  tf2::fromMsg(first_point.point.wrench, first_vector);
  tf2::fromMsg(second_point.point.wrench, second_vector);
  linearly_interpolate(first_vector, second_vector, t_normalized, interpolated_vector);
  tf2::toMsg(interpolated_vector, interpolated_point.wrench);
}

void linearly_interpolate(const acg_control_msgs::msg::TaskSpaceTrajectoryPoint& first_point,
                          const acg_control_msgs::msg::TaskSpaceTrajectoryPoint& second_point, const double t,
                          acg_control_msgs::msg::TaskSpacePoint& interpolated_point)
{
  const double first_timestamp{ rclcpp::Duration(first_point.time_from_start).seconds() };
  const double second_timestamp{ rclcpp::Duration(second_point.time_from_start).seconds() };

  if (first_timestamp >= second_timestamp)
  {
    throw std::runtime_error("The first timestamp should be smaller than the second timestamp");
  }

  if (t < first_timestamp || t > second_timestamp)
  {
    throw std::runtime_error("Time t (" + std::to_string(t) + ") is out of bounds. It should be between " + std::to_string(first_timestamp) +
                             " and " + std::to_string(second_timestamp));
  }

  const double t_normalized{ (t - first_timestamp) / (second_timestamp - first_timestamp) };

  // Declare Eigen vectors as temporary variables to perform linear interpolation
  Eigen::Vector3d first_vector3, second_vector3, interpolated_vector3;

  // Position interpolation
  tf2::fromMsg(first_point.point.pose.position, first_vector3);
  tf2::fromMsg(second_point.point.pose.position, second_vector3);
  linearly_interpolate(first_vector3, second_vector3, t_normalized, interpolated_vector3);
  interpolated_point.pose.position = tf2::toMsg(interpolated_vector3);

  // Orientation interpolation
  Eigen::Quaterniond first_orientation, second_orientation;
  tf2::fromMsg(first_point.point.pose.orientation, first_orientation);
  tf2::fromMsg(second_point.point.pose.orientation, second_orientation);
  interpolated_point.pose.orientation = tf2::toMsg(first_orientation.slerp(t_normalized, second_orientation));

  // Twist interpolation
  Eigen::Matrix<double, 6, 1> first_vector6, second_vector6, interpolated_vector6;
  tf2::fromMsg(first_point.point.twist, first_vector6);
  tf2::fromMsg(second_point.point.twist, second_vector6);
  linearly_interpolate(first_vector6, second_vector6, t_normalized, interpolated_vector6);
  interpolated_point.twist = tf2::toMsg(interpolated_vector6);

  // Acceleration interpolation
  tf2::fromMsg(first_point.point.acceleration, first_vector6);
  tf2::fromMsg(second_point.point.acceleration, second_vector6);
  linearly_interpolate(first_vector6, second_vector6, t_normalized, interpolated_vector6);
  tf2::toMsg(interpolated_vector6, interpolated_point.acceleration);

  // Wrench interpolation
  tf2::fromMsg(first_point.point.wrench, first_vector6);
  tf2::fromMsg(second_point.point.wrench, second_vector6);
  linearly_interpolate(first_vector6, second_vector6, t_normalized, interpolated_vector6);
  tf2::toMsg(interpolated_vector6, interpolated_point.wrench);

  // Wrench derivative interpolation
  tf2::fromMsg(first_point.point.wrench_derivative, first_vector6);
  tf2::fromMsg(second_point.point.wrench_derivative, second_vector6);
  linearly_interpolate(first_vector6, second_vector6, t_normalized, interpolated_vector6);
  tf2::toMsg(interpolated_vector6, interpolated_point.wrench_derivative);
}

}  // namespace acg_interpolation
