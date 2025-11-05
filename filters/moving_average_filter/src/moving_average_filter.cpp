/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   moving_average_filter.cpp
 * Author:  Alessio Coone
 * Org.:    UNISA
 * Date:    Oct 28, 2024
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <pluginlib/class_list_macros.hpp>
#include "moving_average_filter/moving_average_filter.hpp"

namespace moving_average_filter
{
template <typename T>
MovingAverageFilter<T>::MovingAverageFilter() : logger_(rclcpp::get_logger("MovingAverageFilter"))
{}

template <typename T>
MovingAverageFilter<T>::~MovingAverageFilter()
{}

template <typename T>
bool MovingAverageFilter<T>::configure()
{
  if (!params_interface_->has_parameter(param_prefix_ + "number_of_observations"))
  {
    RCLCPP_ERROR(logger_, "Could not retrieve parameter 'number_of_observations' from the node's parameters");
    return false;
  }
  number_of_observations_ = params_interface_->get_parameter(param_prefix_ + "number_of_observations").as_int();

  if (number_of_observations_ < 1)
  {
    RCLCPP_ERROR(logger_, "Parameter 'number_of_observations' should be greater than or equal to 1.");
    return false;
  }

  // Also reset the class member variables
  cumulator_ = T{};

  // Delete any object that was previously pointed to by data_
  data_ = std::make_unique<filters::RealtimeCircularBuffer<T>>(number_of_observations_, T{});
  return true;
}

template <typename T>
bool MovingAverageFilter<T>::update(const T& data_in, T& data_out)
{
  // Assert that the filter was previously configured
  if (!configured_)
  {
    RCLCPP_ERROR(logger_, "The filter must be configured before using the update() method.");
    return false;
  }

  // Check if an element needs to be removed from storage
  T removed_element(data_->size() != number_of_observations_ ? T{} : data_->front());

  // Add new element to the storage
  data_->push_back(data_in);

  // Update cumulator and result
  cumulator_ = cumulator_ - removed_element + data_in;
  data_out = cumulator_ / data_->size();

  return true;
}
}  // namespace moving_average_filter

PLUGINLIB_EXPORT_CLASS(moving_average_filter::MovingAverageFilter<double>, filters::FilterBase<double>)
