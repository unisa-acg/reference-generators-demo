/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   multi_channel_moving_average_filter.cpp
 * Author:  Alessio Coone
 * Org.:    UNISA
 * Date:    Sep 30, 2024
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <pluginlib/class_list_macros.hpp>
#include "multi_channel_moving_average_filter/multi_channel_moving_average_filter.hpp"

namespace moving_average_filter
{
template <typename T>
MultiChannelMovingAverageFilter<T>::MultiChannelMovingAverageFilter() : logger_(rclcpp::get_logger("MultiChannelMovingAverageFilter"))
{}

template <typename T>
MultiChannelMovingAverageFilter<T>::~MultiChannelMovingAverageFilter()
{}

template <typename T>
bool MultiChannelMovingAverageFilter<T>::configure()
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

  // Resize helper variables for the right number of channels
  cumulator_.resize(number_of_channels_, T{});

  // Initialize the cumulator to zero
  cumulator_.assign(number_of_channels_, T{});

  // Delete any object that was previously pointed to by data_
  data_ = std::make_unique<filters::RealtimeCircularBuffer<std::vector<T>>>(number_of_observations_, std::vector<T>(number_of_channels_, T{}));
  return true;
}

template <typename T>
bool MultiChannelMovingAverageFilter<T>::update(const std::vector<T>& data_in, std::vector<T>& data_out)
{
  if (data_in.size() != number_of_channels_ || data_out.size() != number_of_channels_)
  {
    RCLCPP_ERROR(logger_, "Input and output parameters with sizes %lu and %lu differ from configuration: %lu.", data_in.size(), data_out.size(),
                 number_of_channels_);
    return false;
  }

  if (!configured_)
  {
    RCLCPP_ERROR(logger_, "The filter must be configured before using the update() method.");
    return false;
  }

  // Check if an element needs to be removed from storage
  std::vector<T> removed_element = data_->size() != number_of_observations_ ? std::vector(number_of_channels_, T{}) : data_->front();

  // Add new observation to storage
  data_->push_back(data_in);

  // Update cumulators and output for each channel
  for (std::size_t i = 0; i < number_of_channels_; i++)
  {
    cumulator_[i] = cumulator_[i] - removed_element[i] + data_in[i];
    data_out[i] = cumulator_[i] / data_->size();
  }

  return true;
}

}  // namespace moving_average_filter

PLUGINLIB_EXPORT_CLASS(moving_average_filter::MultiChannelMovingAverageFilter<double>, filters::MultiChannelFilterBase<double>)
