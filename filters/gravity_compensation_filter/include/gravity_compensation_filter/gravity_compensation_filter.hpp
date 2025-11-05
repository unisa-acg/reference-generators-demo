/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   gravity_compensation_filter.hpp
 * Author:  Alessio Coone, Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Oct 7, 2024
 *
 * This module provides the
 * gravity_compensation_filter::GravityCompensationFilter class,
 * a templated filter designed for processing multi-channel
 * numeric data. It is designed to compensate the payload gravity
 * bias affecting wrench measurements.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <filters/filter_base.hpp>
#include <eigen3/Eigen/Core>

namespace gravity_compensation_filter
{

/**
 * @class GravityCompensationFilter
 * @brief A class representing a gravity compensation filter.
 *
 * This class implements the \c filters::MultiChannelFilterBase<T> base class and it is designed to compensate the payload gravity
 * bias affecting wrench measurements.
 */
template <typename T>
class GravityCompensationFilter : public filters::MultiChannelFilterBase<T>
{
public:
  /**
   * @brief Number of axes of the force/torque sensor.
   */
  static const unsigned int FT_SENSOR_AXES = 6;

  /**
   * @brief Constructs a GravityCompensationFilter object.
   */
  GravityCompensationFilter();

  /**
   * @brief Destroy a GravityCompensationFilter object.
   */
  ~GravityCompensationFilter();

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
   * @brief Flag indicating if the filter was previously configured.
   */
  using filters::MultiChannelFilterBase<T>::configured_;

  /**
   * @brief Number of parallel inputs for which the filter is to be configured.
   */
  using filters::MultiChannelFilterBase<T>::number_of_channels_;

  /**
   * @brief Pointer to the parameter interface for the filter.
   */
  using filters::MultiChannelFilterBase<T>::params_interface_;

  /**
   * @brief Parameter prefix for the filter.
   */
  using filters::MultiChannelFilterBase<T>::param_prefix_;

  /**
   * @brief Logger instance for the GravityCompensationFilter class.
   */
  rclcpp::Logger logger_;

  /**
   * @brief Force contribution of the payload expressed with respect to the base frame.
   */
  Eigen::Vector3d payload_force_;

  /**
   * @brief Center of mass (COM) of the payload expressed with respect to the force/torque sensor frame.
   */
  Eigen::Vector3d payload_com_;
};

}  // namespace gravity_compensation_filter
