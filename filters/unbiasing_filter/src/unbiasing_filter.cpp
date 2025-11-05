/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   unbiasing_filter.cpp
 * Author:  Alessio Coone, Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Sep 30, 2024
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include "unbiasing_filter/unbiasing_filter.hpp"

namespace unbiasing_filter
{
template <typename T>
UnbiasingFilter<T>::UnbiasingFilter() : logger_(rclcpp::get_logger("unbiasing_filter")), stable_readings_(0), is_bias_computed_(false)
{
  filter_ = std::make_shared<moving_average_filter::MultiChannelMovingAverageFilter<T>>();

  parameter_node_ = std::make_shared<rclcpp::Node>("multi_channel_moving_average_filter");

  parameter_node_->declare_parameter("number_of_observations", 10);
}

template <typename T>
UnbiasingFilter<T>::~UnbiasingFilter()
{
  filter_.reset();
}

template <typename T>
bool UnbiasingFilter<T>::configure()
{
  if (!params_interface_->has_parameter(param_prefix_ + "number_of_stable_samples_for_bias"))
  {
    RCLCPP_ERROR(logger_, "Could not retrieve parameter 'number_of_stable_samples_for_bias' from the node's parameters");
    return false;
  }
  number_of_stable_samples_for_bias_ = params_interface_->get_parameter(param_prefix_ + "number_of_stable_samples_for_bias").as_int();

  if (number_of_stable_samples_for_bias_ < 1)
  {
    RCLCPP_ERROR(logger_, "Parameter 'number_of_stable_samples_for_bias' should be greater than or equal to 1.");
    return false;
  }

  if (!params_interface_->has_parameter(param_prefix_ + "tolerance"))
  {
    RCLCPP_ERROR(logger_, "Could not retrieve parameter 'tolerance' from the node's parameters");
    return false;
  }
  tolerance_ = params_interface_->get_parameter(param_prefix_ + "tolerance").as_double();

  if (tolerance_ < 0.0)
  {
    RCLCPP_ERROR(logger_, "Parameter 'tolerance' should be greater than or equal to 0.0.");
    return false;
  }

  parameter_node_->set_parameter(rclcpp::Parameter("number_of_observations", static_cast<int>(number_of_stable_samples_for_bias_)));

  // Configure the multi-channel moving average filter
  // If the filter is already configured, reset it
  if (configured_)
  {
    if (!filter_->configure())
    {
      RCLCPP_ERROR(logger_, "Could not configure moving average filter.");
      return false;
    }
  }
  else
  {
    if (!filter_->configure(number_of_channels_ - 1, "", "MultiChannelMovingAverageFilter", parameter_node_->get_node_logging_interface(),
                            parameter_node_->get_node_parameters_interface()))
    {
      RCLCPP_ERROR(logger_, "Could not configure moving average filter.");
      return false;
    }
  }

  // Initialize the bias vector
  bias_ = std::vector<T>(number_of_channels_ - 1, T{});

  // Initialize to zero the previous readings
  previous_reading_ = std::vector<T>(number_of_channels_ - 1, T{});

  // Set the bias computed flag to false
  is_bias_computed_ = false;

  // Reset the stable readings counter
  stable_readings_ = 0;

  return true;
}

template <typename T>
bool UnbiasingFilter<T>::update(const std::vector<T>& data_in, std::vector<T>& data_out)
{
  if (!configured_)
  {
    RCLCPP_ERROR(logger_, "The filter must be configured before calling the update() method.");
    return false;
  }

  if (data_in.size() != number_of_channels_)
  {
    RCLCPP_ERROR(logger_, "Input data size is %lu, while the expected size is %lu (with control flag).", data_in.size(), number_of_channels_);
    return false;
  }

  // Copy the input data to the output data to avoid modifying the unused channels
  data_out = data_in;

  // Extract the readings from the input data
  std::vector<T> biased_reading(data_in.begin(), data_in.begin() + number_of_channels_ - 1);

  if (!is_bias_computed_)
  {
    // Verify that the biased readings are not zero
    bool non_zero_measure = std::any_of(biased_reading.begin(), biased_reading.end(), [&](T i) { return std::fabs(i - T{}) > 1e-6; });

    // Verify that the biased readings are equal to the previous readings within the tolerance
    bool equal_to_previous =
        std::equal(biased_reading.begin(), biased_reading.end(), previous_reading_.begin(), [&](T i, T j) { return std::fabs(i - j) <= tolerance_; });

    if (non_zero_measure && equal_to_previous)
    {
      stable_readings_++;

      if (!filter_->update(biased_reading, bias_))
      {
        RCLCPP_ERROR(logger_, "Could not compute average during bias calculation.");
        return false;
      }
    }
    else
    {
      stable_readings_ = 0;

      if (!filter_->configure())
      {
        RCLCPP_ERROR(logger_, "Could not configure moving average filter during bias calculation.");
        return false;
      }
    }

    // Update the previous reading
    previous_reading_ = biased_reading;

    // Set the output data to 0.0 since the sensor bias is not yet computed
    data_out[number_of_channels_ - 1] = T{};

    if (stable_readings_ >= number_of_stable_samples_for_bias_)
    {
      RCLCPP_INFO(logger_, "Bias computed");
      data_out[number_of_channels_ - 1] = T{ 1 };
      is_bias_computed_ = true;
    }
  }

  for (size_t i = 0; i < number_of_channels_ - 1; ++i)
  {
    data_out[i] = biased_reading[i] - bias_[i];
  }

  return true;
}
}  // namespace unbiasing_filter

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(unbiasing_filter::UnbiasingFilter<double>, filters::MultiChannelFilterBase<double>)
