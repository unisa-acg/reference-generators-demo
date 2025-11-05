/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   moving_average_poses_filter.hpp
 * Author:  Michele Marsico, Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Oct 24, 2024
 *
 * This module provides the
 * moving_average_poses_filter::MovingAveragePosesFilter class,
 * a templated filter designed for processing multi-channel
 * numeric data. It is designed to compute the moving average
 * of poses.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <filters/filter_base.hpp>
#include <filters/realtime_circular_buffer.hpp>

template <typename T>
using Matrix4x = typename Eigen::Matrix<T, 4, 4, Eigen::RowMajor>;

template <typename T>
using Vector4x = typename Eigen::Matrix<T, 4, 1>;

namespace moving_average_poses_filter
{

/**
 * @class MovingAveragePosesFilter
 * @brief A class representing a moving average poses filter.
 *
 * This class implements the \c filters::MultiChannelFilterBase<T> base class and it is designed to compute the moving average
 * of poses, expressed as standard vectors. It uses a circular buffer to store the last N observations and an accumulator
 * to compute the mean efficiently without iterating through all elements each time.
 */
template <typename T>
class MovingAveragePosesFilter : public filters::MultiChannelFilterBase<T>
{
public:
  /**
   * @brief Number of channels for the position.
   */
  static const unsigned int POSITION_CHANNELS = 3;

  /**
   * @brief Number of channels for the orientation.
   */
  static const unsigned int ORIENTATION_CHANNELS = 4;

  /**
   * @brief Number of channels for the cumulator. The latter stores the squared norm of the orientation quaternions.
   */
  static const unsigned int CUMULATOR_CHANNELS = POSITION_CHANNELS + (ORIENTATION_CHANNELS * ORIENTATION_CHANNELS);

  /**
   * @brief Construct a MovingAveragePosesFilter object.
   */
  MovingAveragePosesFilter();

  /**
   * @brief Destroy a MovingAveragePosesFilter object.
   */
  ~MovingAveragePosesFilter();

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
   * @brief Logger instance for the MovingAveragePosesFilter class.
   */
  rclcpp::Logger logger_;
};

}  // namespace moving_average_poses_filter
