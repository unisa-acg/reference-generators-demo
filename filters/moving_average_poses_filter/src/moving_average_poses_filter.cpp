/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   moving_average_poses_filter.cpp
 * Author:  Michele Marsico, Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Oct 24, 2024
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <pluginlib/class_list_macros.hpp>

#include <Eigen/SVD>

#include "moving_average_poses_filter/moving_average_poses_filter.hpp"

using namespace moving_average_poses_filter;

template <typename T>
MovingAveragePosesFilter<T>::MovingAveragePosesFilter() : logger_(rclcpp::get_logger("MovingAveragePosesFilter"))
{}

template <typename T>
MovingAveragePosesFilter<T>::~MovingAveragePosesFilter()
{}

template <typename T>
bool MovingAveragePosesFilter<T>::configure()
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

  if (number_of_channels_ != POSITION_CHANNELS + ORIENTATION_CHANNELS)
  {
    RCLCPP_ERROR(logger_, "The number of channels should be equal to 7: 3 for the position and 4 for the orientation");
    return false;
  }

  // Resize helper variables for the right number of channels
  cumulator_.resize(CUMULATOR_CHANNELS, T{});

  // Initialize the cumulator to zero
  cumulator_.assign(CUMULATOR_CHANNELS, T{});

  // Delete any object that was previously pointed to by data_
  data_ = std::make_unique<filters::RealtimeCircularBuffer<std::vector<T>>>(number_of_observations_, std::vector<T>(number_of_channels_, T{}));
  return true;
}

template <typename T>
bool MovingAveragePosesFilter<T>::update(const std::vector<T>& data_in, std::vector<T>& data_out)
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

  // Check if an element needs to be removed from storages
  std::vector<T> removed_element = data_->size() != number_of_observations_ ? std::vector(CUMULATOR_CHANNELS, T{}) : data_->front();

  // Add new observation to storage
  Vector4x<T> q(data_in[3], data_in[4], data_in[5], data_in[6]);
  Matrix4x<T> q_square_norm = q * q.transpose();

  std::vector<T> input_data(CUMULATOR_CHANNELS, T{});
  std::copy(data_in.begin(), data_in.begin() + POSITION_CHANNELS, input_data.begin());
  std::copy(q_square_norm.data(), q_square_norm.data() + q_square_norm.rows() * q_square_norm.cols(), input_data.begin() + POSITION_CHANNELS);

  data_->push_back(input_data);

  // Initialize the normalized cumulator
  std::vector<T> normalized_cumulator(CUMULATOR_CHANNELS, T{});

  // Update cumulators and output for each channel
  for (std::size_t i = 0; i < CUMULATOR_CHANNELS; i++)
  {
    cumulator_[i] = cumulator_[i] - removed_element[i] + input_data[i];
    normalized_cumulator[i] = cumulator_[i] / data_->size();
  }

  /* Based on:
   F. L. Markley, Y. Cheng, J. L. Crassidis, and Y. Oshman.
   "Averaging quaternions." Journal of Guidance, Control, and Dynamics 30,
   no. 4 (2007): 1193-1197. Link: https://ntrs.nasa.gov/citations/20070017872 */
  Matrix4x<T> M = Eigen::Map<Matrix4x<T>>(normalized_cumulator.data() + POSITION_CHANNELS, ORIENTATION_CHANNELS, ORIENTATION_CHANNELS);

  // Select the eigenvector corresponding to the greatest eigenvalue of M
  Eigen::JacobiSVD<Matrix4x<T>> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix4x<T> U = svd.matrixU();

  // Update output for each channel
  for (size_t i = 0; i < POSITION_CHANNELS; i++)
  {
    data_out[i] = normalized_cumulator[i];
  }

  for (std::size_t i = POSITION_CHANNELS; i < number_of_channels_; i++)
  {
    data_out[i] = U(i - POSITION_CHANNELS, 0);
  }

  return true;
}

PLUGINLIB_EXPORT_CLASS(moving_average_poses_filter::MovingAveragePosesFilter<double>, filters::MultiChannelFilterBase<double>)
