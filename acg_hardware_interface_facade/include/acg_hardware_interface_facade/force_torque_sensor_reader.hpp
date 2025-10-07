/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   force_torque_sensor_reader.hpp
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Apr 09, 2025
 *
 * This module contains the ForceTorqueSensorReader class, which is
 * used to read from the state interfaces.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <vector>
#include <string>

#include <hardware_interface/loaned_state_interface.hpp>
#include <semantic_components/force_torque_sensor.hpp>
#include <geometry_msgs/msg/wrench.hpp>

namespace acg_hardware_interface_facade
{
/**
 * @brief Class that handles the reading of the force/torque sensor state interfaces, based on the state interfaces specified in the configuration
 * step.
 */
class ForceTorqueSensorReader
{
public:
  /**
   * @brief Construct a new force/torque sensor reader object. This function is not real-time safe.
   */
  ForceTorqueSensorReader() = default;

  /**
   * @brief Destruct this object. This function is not real-time safe.
   */
  ~ForceTorqueSensorReader() = default;

  /**
   * @brief Configure the state interfaces for the force/torque sensor reader.
   *
   * This function must be called before attempting to read state interfaces. It sets up the internal mapping
   * of state interface names required for reading wrench data from the sensor.
   *
   * By default, the interface names are generated using the following naming convention:
   *   <sensor_name>/<component>
   * where <sensor_name> is the name of the f/t sensor and <component> is one of:
   *   {"force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"}.
   *
   * Alternatively, the user may provide a custom list of interface names via the \c state_interface_names_override parameter.
   *
   * @param[in] sensor_name Name of the force/torque sensor.
   * @param[in] state_interface_names_override Optional vector of custom state interface names to use instead of the default.
   * @return true if configuration succeeds, false otherwise.
   */
  bool configure_state_interfaces(const std::string& sensor_name, const std::vector<std::string>& state_interface_names_override = {});

  /**
   * @brief Get the state interfaces names.
   *
   * This function returns the state interfaces names based on the information specified via the \c configure_state_interfaces function.
   * This function is not real-time safe.
   *
   * @return a vector of strings containing the state interfaces names.
   */
  std::vector<std::string> available_state_interfaces() const;

  /**
   * @brief Read the state interfaces.
   *
   * This function reads the state interfaces specified via the \c configure_state_interfaces function and stores
   * the state in the \c state variable. This function is real-time safe.
   *
   * @param[out] state The message to read the wrench state into.
   */
  void read_state_interfaces(geometry_msgs::msg::Wrench& state) const;

  /**
   * @brief Assign the state interfaces to the force/torque sensor reader. This function should be called once during the controller activation step.
   *
   * This function assigns the state interfaces to the force/torque sensor reader, based on the state interfaces specified in the configuration
   * step. This function is not real-time safe.
   *
   * @param[in] state_interfaces The vector of loaned state interfaces.
   * @return true if the state interfaces are correctly assigned, false otherwise.
   */
  bool assign_loaned_state_interfaces(std::vector<hardware_interface::LoanedStateInterface>& state_interfaces);

  /**
   * @brief Release the assigned state interfaces. This function is not real-time safe.
   */
  void release_interfaces();

protected:
  std::vector<std::string> interface_names_;

  std::unique_ptr<semantic_components::ForceTorqueSensor> force_torque_sensor_;

  bool configured_{ false };

  /**
   * @brief Logs an error message if the interface is not configured.
   *
   * This function checks whether the interface is configured. If it is not,
   * it logs an appropriate error message. This function also returns a boolean
   * value indicating whether the interface is configured.
   *
   * @return true if the object is configured, false otherwise.
   */
  bool is_configured_() const;
};
;

}  // namespace acg_hardware_interface_facade
