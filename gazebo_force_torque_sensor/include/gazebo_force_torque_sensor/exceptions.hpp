/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   exceptions.hpp
 * Author:  Alessio Coone
 * Org.:    UNISA
 * Date:    Sep 8, 2024
 *
 * Domain specific exceptions for gazebo_force_torque_sensor.
 * This module masks the rclcpp::exceptions::InvalidParametersException
 * class in the Exception class of this namespace/package.
 *
 * -------------------------------------------------------------------
 */

#pragma once
#include "rclcpp/exceptions.hpp"

namespace gz_ros2_control
{
typedef rclcpp::exceptions::InvalidParametersException Exception;
}  // namespace gz_ros2_control
