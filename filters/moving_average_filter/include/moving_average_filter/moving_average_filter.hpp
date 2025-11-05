/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   moving_average_filter.hpp
 * Author:  Alessio Coone
 * Org.:    UNISA
 * Date:    Oct 28, 2024
 *
 * This module implements the filters::FilterBase interface and
 * consists of a templated filter for single-channel numeric data.
 * By using an acummulator variable and a circular buffer, it avoids
 * accessing each element for computing the whole mean as the original
 * filters::FilterMean<T> do.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <filters/filter_base.hpp>
#include <filters/realtime_circular_buffer.hpp>

namespace moving_average_filter
{

/**
 * @class MovingAverageFilter
 * @brief A class representing a moving average filter.
 *
 * This class implements the \c filters::FilterBase<T> base class and it is designed to compute the moving average
 * of a single-channel numeric data stream. It uses a circular buffer to store the last N observations and an accumulator
 * to compute the mean efficiently without iterating through all elements each time.
 */
template <typename T>
class MovingAverageFilter : public filters::FilterBase<T>
{
public:
  /**
   * @brief Construct a MovingAverageFilter object.
   */
  MovingAverageFilter();

  /**
   * @brief Destroy a MovingAverageFilter object.
   */
  ~MovingAverageFilter();

  /**
   * @brief Refer to the superclass documentation.
   */
  bool configure(const std::string& param_prefix, const std::string& filter_name,
                 const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger,
                 const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params)
  {
    if (!configured_)
    {
      return filters::FilterBase<T>::configure(param_prefix, filter_name, node_logger, node_params);
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
  bool update(const T& data_in, T& data_out) override;

private:
  /**
   * @brief Elements within the current moving window.
   */
  std::unique_ptr<filters::RealtimeCircularBuffer<T>> data_;

  /**
   * @brief The maximum number of elements that the moving window can store.
   */
  std::size_t number_of_observations_;

  /**
   * @brief Sum of elements within the current moving window.
   */
  T cumulator_;

  /**
   * @brief Logger instance for the MovingAverageFilter class.
   */
  rclcpp::Logger logger_;

  /**
   * @brief Flag indicating if the filter was previously configured.
   */
  using filters::FilterBase<T>::configured_;

  /**
   * @brief Pointer to the parameter interface for the filter.
   */
  using filters::FilterBase<T>::params_interface_;

  /**
   * @brief Parameter prefix for the filter.
   */
  using filters::FilterBase<T>::param_prefix_;
};

}  // namespace moving_average_filter
