/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   unbiasing_filter.hpp
 * Author:  Alessio Coone, Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Sep 20, 2024
 *
 * This module provides the unbiasing_filter::UnbiasingFilter class,
 * a templated filter designed for processing multi-channel numeric
 * data. It leverages the MultiChannelMovingAverageFilter class to
 * compute the measurement bias. After the bias is calculated,
 * the filter provides unbiased measurements.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <filters/filter_base.hpp>
#include <multi_channel_moving_average_filter/multi_channel_moving_average_filter.hpp>

namespace unbiasing_filter
{

/**
 * @class UnbiasingFilter
 * @brief A class representing an unbiasing filter.
 *
 * This class implements the \c filters::MultiChannelFilterBase<T> base class and it is designed to compute the bias of a multi-channel
 * numeric data stream using a moving average filter. After the bias is calculated, it provides unbiased measurements.
 */
template <typename T>
class UnbiasingFilter : public filters::MultiChannelFilterBase<T>
{
public:
  /**
   * @brief Construct an UnbiasingFilter object.
   */
  UnbiasingFilter();

  /**
   * @brief Destroy an UnbiasingFilter object.
   */
  ~UnbiasingFilter();

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
   * @brief Shared pointer to the ROS2 node used for parameter handling.
   */
  std::shared_ptr<rclcpp::Node> parameter_node_;

  /**
   * @brief Logger instance for the UnbiasingFilter class.
   */
  rclcpp::Logger logger_;

  /**
   * @brief Pointer to an instance of the multi-channel moving average filter class.
   * This filter is responsible for computing the average of stable measurements, i.e. the bias
   */
  std::shared_ptr<moving_average_filter::MultiChannelMovingAverageFilter<T>> filter_;

  /**
   * @brief The number of stable samples required to calculate the bias.
   */
  unsigned int number_of_stable_samples_for_bias_;

  /**
   * @brief The number of stable readings observed.
   */
  unsigned int stable_readings_;

  /**
   * @brief Stores the measurements from the previous cycle or function call.
   */
  std::vector<T> previous_reading_;

  /**
   * @brief The bias computed for each axis.
   */
  std::vector<T> bias_;

  /**
   * @brief The maximum allowable difference between two values.
   */
  T tolerance_;

  /**
   * @brief Flag indicating if the bias has been calculated.
   */
  bool is_bias_computed_;
};

}  // namespace unbiasing_filter
