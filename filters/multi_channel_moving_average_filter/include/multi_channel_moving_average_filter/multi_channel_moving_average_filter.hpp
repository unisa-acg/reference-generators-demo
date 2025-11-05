/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   multi_channel_moving_average_filter.hpp
 * Author:  Alessio Coone
 * Org.:    UNISA
 * Date:    Sep 30, 2024
 *
 * This module implements the filters::MultiChannelFilterBase
 * interface and consists of a templated filter for multi-channel
 * numeric data. By using an acummulator variable and a circular
 * buffer, it avoids accessing each element for computing the whole
 * mean as the original filters::MultiChannelMean<T> do.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <filters/filter_base.hpp>
#include <filters/realtime_circular_buffer.hpp>

namespace moving_average_filter
{

/**
 * @class MultiChannelMovingAverageFilter
 * @brief A class representing a multi-channel moving average filter.
 *
 * This class implements the \c filters::MultiChannelFilterBase<T> base class and it is designed to compute the moving average
 * of multi-channel numeric data streams. It uses a circular buffer to store the last N observations and an accumulator
 * to compute the mean efficiently without iterating through all elements each time.
 */
template <typename T>
class MultiChannelMovingAverageFilter : public filters::MultiChannelFilterBase<T>
{
public:
  /**
   * @brief Construct a MultiChannelMovingAverageFilter object.
   */
  MultiChannelMovingAverageFilter();

  /**
   * @brief Destroy a MultiChannelMovingAverageFilter object.
   */
  ~MultiChannelMovingAverageFilter();

  /**
   * @brief Refer to the superclass documentation.
   */
  bool configure(size_t number_of_channels, const std::string& param_prefix, const std::string& filter_name,
                 const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger,
                 const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params)
  {
    if (!configured_)
    {
      return filters::MultiChannelFilterBase<T>::configure(number_of_channels, param_prefix, filter_name, node_logger, node_params);
    }
    return true;
  }

  /**
   * @brief Refer to the superclass documentation.
   *
   * This function can be used to reset the filter after it has been properly configured.
   */
  bool configure() override;

  /**
   * @brief Refer to the superclass documentation.
   */
  bool update(const std::vector<T>& data_in, std::vector<T>& data_out) override;

private:
  /**
   * @brief Elements within the current moving window.
   */
  std::unique_ptr<filters::RealtimeCircularBuffer<std::vector<T>>> data_;

  /**
   * @brief The maximum number of elements that the moving window can store.
   */
  std::size_t number_of_observations_;

  /**
   * @brief Number of parallel inputs for which the filter is to be configured.
   */
  using filters::MultiChannelFilterBase<T>::number_of_channels_;

  /**
   * @brief Flag indicating if the filter was previously configured.
   */
  using filters::MultiChannelFilterBase<T>::configured_;

  /**
   * @brief Pointer to the parameter interface for the filter.
   */
  using filters::MultiChannelFilterBase<T>::params_interface_;

  /**
   * @brief Parameter prefix for the filter.
   */
  using filters::MultiChannelFilterBase<T>::param_prefix_;

  /**
   * @brief Sum of elements within the current moving window.
   */
  std::vector<T> cumulator_;

  /**
   * @brief Logger instance for the MultiChannelMovingAverageFilter class.
   */
  rclcpp::Logger logger_;
};

}  // namespace moving_average_filter
