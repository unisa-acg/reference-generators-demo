/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   interaction_filter_base.hpp
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Apr 15, 2025
 *
 * This module defines an abstract class for interaction filters.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <acg_control_msgs/msg/task_space_point.hpp>

namespace interaction_filters
{
class InteractionFilterBase
{
public:
  /**
   * @brief Virtual destructor.
   */
  virtual ~InteractionFilterBase() = default;

  /**
   * @brief Initialize the filter.
   * @param[in] parameters_interface Pointer to the parameter interface for the filter.
   * @param[in] logging_interface Pointer to the node logging interface for the filter.
   * @param[in] filter_name Name of the filter.
   * @return True if configuration succeeds, false otherwise.
   */
  bool initialize(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& parameters_interface,
                  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface, const std::string& filter_name)
  {
    // Check if the filter is already initialized
    if (is_initialized())
    {
      RCLCPP_WARN(logging_interface_->get_logger(), "Filter %s is already initialized", filter_name_.c_str());
      return true;
    }

    // Store the parameters interface, logger, and filter name
    params_interface_ = parameters_interface;
    logging_interface_ = logging_interface;
    filter_name_ = filter_name;

    // Initialize the filter
    initialized_ = initialize_();
    return is_initialized();
  }

  /**
   * @brief Update the filter.
   * @param[in] task_space_reference Task space reference for the filter.
   * @param[in] delta_t Interval of time between two consecutive updates.
   * @param[out] task_space_command Filter command.
   * @return True if update succeeds, false otherwise.
   */
  virtual bool update(const acg_control_msgs::msg::TaskSpacePoint& task_space_reference, const rclcpp::Duration& delta_t,
                      acg_control_msgs::msg::TaskSpacePoint& task_space_command) = 0;

  /**
   * @brief Reset the filter.
   * @return True if reset succeeds, false otherwise.
   */
  virtual bool reset() = 0;

  /**
   * @brief Check whether the filter has been initialized.
   * @return True if initialized, false otherwise.
   */
  bool is_initialized() const
  {
    return initialized_;
  }

  /**
   * @brief Update parameters if any parameters have changed since last update.
   */
  virtual void apply_parameters_update() = 0;

protected:
  /**
   * @brief Pure virtual function for the derived class to initialize the filter.
   * @return True if initialization succeeds, false otherwise.
   */
  virtual bool initialize_() = 0;

  /**
   * @brief Flag indicating whether the filter is initialized or not.
   */
  bool initialized_{ false };

  /**
   * @brief Pointer to the parameter interface for the filter.
   */
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params_interface_;

  /**
   * @brief Pointer to the node logging interface for the filter.
   */
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_;

  /**
   * @brief Name of the filter.
   */
  std::string filter_name_;
};

}  // namespace interaction_filters
