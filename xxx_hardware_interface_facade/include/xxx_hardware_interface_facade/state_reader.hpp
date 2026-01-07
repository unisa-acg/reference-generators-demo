/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   state_reader.hpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Jan 30, 2025
 *
 * This module contains the StateReader class, which is used to read
 * from the state interfaces.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <vector>
#include <string>

#include <hardware_interface/loaned_state_interface.hpp>

#include <xxx_semantic_components/position_encoder.hpp>
#include <xxx_semantic_components/velocity_encoder.hpp>
#include <xxx_semantic_components/accelerometer.hpp>
#include <xxx_semantic_components/effort_sensor.hpp>

namespace xxx_hardware_interface_facade
{

/**
 * @brief Struct that contains the robot joint state. The struct contains the positions, velocities, accelerations, and efforts of the robot joints.
 */
struct RobotJointState
{
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
  std::vector<double> efforts;
};

/**
 * This struct aggregates the names of all state interfaces that the user wants to override. It is used to configure the StateReader class.
 *
 * The user is responsible for providing the correct number of names in each vector: each vector must contain a number of names equal to the number of
 * joints in the robot and in the same order as the information will be retrieved.
 *
 * By default, the state interfaces are named according to the following convention:
 * <robot_name>/<joint_name>/<state_interface_type>
 * where:
 * - robot_name is the name of the robot;
 * - joint_name is the name of the joint;
 * - state_interface_type is the type of the state interface, which can be "position", "velocity", "acceleration", "effort".
 */
struct StateInterfaceNamesOverrideConfig
{
  std::vector<std::string> position_state_interfaces;
  std::vector<std::string> velocity_state_interfaces;
  std::vector<std::string> acceleration_state_interfaces;
  std::vector<std::string> effort_state_interfaces;
};

/**
 * @brief Class that handles the reading of the state interfaces, based on the state interfaces specified in the configuration step.
 */
class StateReader
{
public:
  /**
   * @brief Construct a new state reader object. This function is not real-time safe.
   */
  StateReader() = default;

  /**
   * @brief Destruct this object. This function is not real-time safe.
   */
  ~StateReader() = default;

  /**
   * @brief Configure the state interfaces. This function must be called before reading the state interfaces.
   *
   * This function configures the state interfaces that the \c StateReader will access ("position", "velocity", "acceleration", "effort").
   * For each selected interface type, an internal semantic component (encoder/sensor) is created to manage its data.
   * By default, the fully-qualified name of each state interface is built using the template: <robot_name>/<joint_name>/<state_interface>.
   * For example: `"my_robot/joint1/position"`.
   * The user can override the standard naming convention with custom names through the \c StateInterfaceNamesOverrideConfig struct.
   * This function is not real-time safe.
   *
   * @param[in] state_interfaces The state interfaces the user wants to read, that can be "position", "velocity", "acceleration", "effort".
   * @param[in] joint_names The joint names. Used to compute the number of joints, as well as to build the default state interface names.
   * @param[in] robot_name The robot name.
   * @param[in] state_interface_names_override The struct that contains the names of the state interfaces that the user wants to override.
   * The names specified in the struct must be coherent with the size of the joint_names vector, otherwise the default names will be used.
   *
   * @return true if the object is correctly configured, false otherwise.
   */
  bool configure_state_interfaces(const std::vector<std::string>& state_interfaces, const std::vector<std::string>& joint_names,
                                  const std::string& robot_name, const StateInterfaceNamesOverrideConfig& state_interface_names_override = {});

  /**
   * @brief Get the state interfaces names.
   *
   * This function returns the state interfaces names based on the information specified via the \c configure_state_interfaces method.
   * This function is not real-time safe.
   *
   * @return a vector of strings containing the state interfaces names.
   */
  std::vector<std::string> available_state_interfaces() const;

  /**
   * @brief Read the state interfaces.
   *
   * This function reads the state interfaces specified via the \c configure_state_interfaces function and stores the state in the state variable.
   * This function is real-time safe.
   *
   * @param[out] state The state to read the state interfaces to. Be sure to have assigned the correct sizes to the fields
   * of the output parameter state, otherwise the function won't read the state.
   */
  void read_state_interfaces(RobotJointState& state) const;

  /**
   * @brief Check if the state reader is configured to read the joint position state interfaces. This function is real-time safe.
   *
   * @return true if the state reader is configured to read the joint position state interfaces, false otherwise.
   */
  bool has_joint_position_state_interface() const;

  /**
   * @brief Check if the state reader is configured to read the joint velocity state interfaces. This function is real-time safe.
   *
   * @return true if the state reader is configured to read the joint velocity state interfaces, false otherwise.
   */
  bool has_joint_velocity_state_interface() const;

  /**
   * @brief Check if the state reader is configured to read the joint acceleration state interfaces. This function is real-time safe.
   *
   * @return true if the state reader is configured to read the joint acceleration state interfaces, false otherwise.
   */
  bool has_joint_acceleration_state_interface() const;

  /**
   * @brief Check if the state reader is configured to read the joint effort state interfaces. This function is real-time safe.
   *
   * @return true if the state reader is configured to read the joint effort state interfaces, false otherwise.
   */
  bool has_joint_effort_state_interface() const;

  /**
   * @brief Assign the state interfaces to the state reader. This function should be called once during the controller activation step.
   *
   * This function assigns the state interfaces to the state reader, based on the state interfaces specified in the configuration step.
   * This function is not real-time safe.
   *
   * @param[in] state_interfaces The vector of loaned state interfaces.
   *
   * @return true if the state interfaces are correctly assigned, false otherwise.
   */
  bool assign_loaned_state_interfaces(std::vector<hardware_interface::LoanedStateInterface>& state_interfaces);

  /**
   * @brief Release the state interfaces held by this class.
   *
   * This method should be called within the \p on_deactivate() method of a chainable controller.
   * In a controller chain, activating a new controller triggers the \p on_deactivate() and \p on_activate() methods of all subsequent controllers in
   * the chain. Each controller must release any interfaces it holds. Therefore, any interfaces assigned to this class should be released in this
   * method.
   */
  void release_interfaces();

protected:
  // internal variables to handle the state interfaces
  std::vector<std::string> joint_names_;
  std::size_t num_joints_{ 0 };
  std::string robot_name_;

  bool has_joint_position_state_interface_{ false };
  bool has_joint_velocity_state_interface_{ false };
  bool has_joint_acceleration_state_interface_{ false };
  bool has_joint_effort_state_interface_{ false };

  std::vector<std::string> position_interfaces_names_;
  std::vector<std::string> velocity_interfaces_names_;
  std::vector<std::string> acceleration_interfaces_names_;
  std::vector<std::string> effort_interfaces_names_;

  std::unique_ptr<xxx_semantic_components::PositionEncoder> position_encoder_;
  std::unique_ptr<xxx_semantic_components::VelocityEncoder> velocity_encoder_;
  std::unique_ptr<xxx_semantic_components::Accelerometer> accelerometer_;
  std::unique_ptr<xxx_semantic_components::EffortSensor> effort_sensor_;

  bool configured_{ false };

  /**
   * @brief Returns whether the object is configured or not. If it is not configured, it logs an error message.
   *
   * @return true if the object is configured, false otherwise.
   */
  bool is_configured_() const;
};

}  // namespace xxx_hardware_interface_facade
