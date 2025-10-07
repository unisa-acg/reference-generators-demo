/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   command_interface_handler.hpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Jan 30, 2025
 *
 * This module provides the base class for both the ReferenceReader
 * and CommandWriter classes. It is designed to manage common
 * functionalities shared by these two classes.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <vector>
#include <string>

#include <hardware_interface/loaned_state_interface.hpp>

#include <acg_semantic_components/joint_position_command.hpp>
#include <acg_semantic_components/joint_velocity_command.hpp>
#include <acg_semantic_components/joint_acceleration_command.hpp>
#include <acg_semantic_components/joint_effort_command.hpp>
#include <acg_semantic_components/wrench_command.hpp>
#include <acg_semantic_components/wrench_derivative_command.hpp>
#include <acg_semantic_components/pose_command.hpp>
#include <acg_semantic_components/twist_command.hpp>
#include <acg_semantic_components/accel_command.hpp>

namespace acg_hardware_interface_facade
{

/**
 * This struct aggregates the names of all command interfaces that the user wants to override.
 * It is used to configure the subclasses of the \c CommandInterfaceHandler class, specifically the \c CommandWriter and \c ReferenceReader
 * subclasses.
 *
 * The user is responsible for providing the correct number of names in each vector:
 * - \c joint_position_interface_names, \c joint_velocity_interface_names, \c joint_acceleration_interface_names, and
 *   \c joint_effort_interface_names must each contain a number of names equal to the number of joints in the robot.
 * - \c joint_wrench_interface_names, \c task_space_twist_interface_names, \c task_space_acceleration_interface_names,
 *   \c task_space_wrench_interface_names, and \c task_space_wrench_derivative_interface_names must each contain a number of names equal to
 *   the size of the corresponding semantic component.
 * - \c task_space_pose_interface_names must contain a number of names equal to the size of the corresponding semantic component.
 *
 * Additionally, the user must ensure that the order of the names in the vectors aligns with the corresponding data structure order in the messages
 * used by the \c CommandWriter and \c ReferenceReader classes. For example, the order of the task_space_pose_interface_names must match the order
 * of the data in the \c{geometry_msgs/Pose.msg}: name1 -> x, name2 -> y, name3 -> z, name4 -> qx, name5 -> qy, name6 -> qz, name7 -> qw.
 */

struct CommandInterfaceNamesOverrideConfig
{
  std::vector<std::string> joint_position_interface_names;
  std::vector<std::string> joint_velocity_interface_names;
  std::vector<std::string> joint_acceleration_interface_names;
  std::vector<std::string> joint_effort_interface_names;
  std::vector<std::string> joint_wrench_interface_names;

  std::vector<std::string> task_space_pose_interface_names;
  std::vector<std::string> task_space_twist_interface_names;
  std::vector<std::string> task_space_acceleration_interface_names;
  std::vector<std::string> task_space_wrench_interface_names;
  std::vector<std::string> task_space_wrench_derivative_interface_names;
};

/**
 * @brief Base class that handles common functionalities for the CommandWriter and ReferenceReader classes.
 */
class CommandInterfaceHandler
{
public:
  /**
   * @brief Construct a new command interface handler object. This function is not real-time safe.
   */
  CommandInterfaceHandler() = default;

  /**
   * @brief Destruct the command interface handler object. This function is not real-time safe.
   */
  ~CommandInterfaceHandler() = default;

  /**
   * @brief Configure the object. This function must be called before performing any operation on the command interfaces, e.g. reading or writing.
   * This function is not real-time safe.
   *
   * This function determines the names of the task space and joint space interfaces provided by the controllers, based on the information specified
   * by the user. Default naming convention are defined within the semantic component commands, and the user can override them with custom names
   * through the \p CommandInterfaceNamesOverrideConfig struct.
   *
   * @param[in] joint_names The joint names. Used to retrieve the number of joints, as well as to build the default command interface names.
   * @param[in] joint_space_interfaces The joint space interfaces to configure.
   * @param[in] task_space_interfaces The task space interfaces to configure.
   * @param[in] joint_space_controller_name The name of the joint space controller that provides the joint space interfaces.
   * @param[in] task_space_controller_name The name of the task space controller that provides the task space interfaces.
   * @param[in] override_config The struct that contains the names of the command interfaces that the user wants to override. The names specified in
   * this struct must be coherent with the types declared in the joint_space_interfaces and task_space_interfaces variables and with the size of the
   * joint_names vector or the sizes documented in the struct, otherwise the default names will be used.
   *
   * @return true if the object is correctly configured, false otherwise.
   */
  bool configure_interfaces(const std::vector<std::string>& joint_names, const std::vector<std::string>& joint_space_interfaces,
                            const std::vector<std::string>& task_space_interfaces, const std::string& joint_space_controller_name,
                            const std::string& task_space_controller_name, const CommandInterfaceNamesOverrideConfig& override_config = {});
  /**
   * @brief Get the full names of the interfaces generated according to the configuration. This function is not real-time safe.
   *
   * This function returns the names of the configured interfaces. It is commonly called in the \p on_export_reference_interfaces or
   * \p on_export_command_interfaces methods of the controller.
   *
   * @return A vector of strings containing the full names of the interfaces.
   */
  std::vector<std::string> available_interfaces() const;

  /**
   * @brief Check if the object is configured to handle the joint position interface. This function is real-time safe.
   *
   * @return true if the object is configured with the joint position interface, false otherwise.
   */
  bool has_joint_position_interface() const;

  /**
   * @brief Check if the object is configured to handle the joint velocity interface. This function is real-time safe.
   *
   * @return true if the object is configured with the joint velocity interface, false otherwise.
   */
  bool has_joint_velocity_interface() const;

  /**
   * @brief Check if the object is configured to handle the joint acceleration interface. This function is real-time safe.
   *
   * @return true if the object is configured with the joint acceleration interface, false otherwise.
   */
  bool has_joint_acceleration_interface() const;

  /**
   * @brief Check if the object is configured to handle the joint effort interface. This function is real-time safe.
   *
   * @return true if the object is configured with the joint effort interface, false otherwise.
   */
  bool has_joint_effort_interface() const;

  /**
   * @brief Check if the object is configured to handle the joint wrench interface. This function is real-time safe.
   *
   * @return true if the object is configured with the joint wrench interface, false otherwise.
   */
  bool has_joint_wrench_interface() const;

  /**
   * @brief Check if the object is configured to handle the task space pose interface. This function is real-time safe.
   *
   * @return true if the object is configured with the task space pose interface, false otherwise.
   */
  bool has_task_space_pose_interface() const;

  /**
   * @brief Check if the object is configured to handle the task space twist interface. This function is real-time safe.
   *
   * @return true if the object is configured with the task space twist interface, false otherwise.
   */
  bool has_task_space_twist_interface() const;

  /**
   * @brief Check if the object is configured to handle the task space acceleration interface. This function is real-time safe.
   *
   * @return true if the object is configured with the task space acceleration interface, false otherwise.
   */
  bool has_task_space_acceleration_interface() const;

  /**
   * @brief Check if the object is configured to handle the task space wrench interface. This function is real-time safe.
   *
   * @return true if the object is configured with the task space wrench interface, false otherwise.
   */
  bool has_task_space_wrench_interface() const;

  /**
   * @brief Check if the object is configured to handle the task space wrench derivative interface. This function is real-time safe.
   *
   * @return true if the object is configured with the task space wrench derivative interface, false otherwise.
   */
  bool has_task_space_wrench_derivative_interface() const;

  /**
   * @brief Release the command interfaces held by this class.
   *
   * This method should be called within the \p on_deactivate() method of a chainable controller.
   * In a controller chain, activating a new controller triggers the \p on_deactivate() and \p on_activate() methods of all subsequent controllers in
   * the chain. Each controller must release any interfaces it holds. Therefore, any interfaces assigned to this class should be released in this
   * method.
   */
  virtual void release_interfaces();

protected:
  // internal variables to store the information specified by the user
  std::size_t num_joints_{ 0 };
  std::vector<std::string> joint_names_;
  std::string joint_space_controller_name_;
  std::string task_space_controller_name_;

  // internal variables to store the interface types that the class is configured to handle
  bool has_joint_position_interface_{ false };
  bool has_joint_velocity_interface_{ false };
  bool has_joint_acceleration_interface_{ false };
  bool has_joint_effort_interface_{ false };
  bool has_joint_wrench_interface_{ false };

  bool has_task_space_pose_interface_{ false };
  bool has_task_space_twist_interface_{ false };
  bool has_task_space_acceleration_interface_{ false };
  bool has_task_space_wrench_interface_{ false };
  bool has_task_space_wrench_derivative_interface_{ false };

  // internal variables to store the interface names
  std::vector<std::string> joint_position_interface_names_;
  std::vector<std::string> joint_velocity_interface_names_;
  std::vector<std::string> joint_acceleration_interface_names_;
  std::vector<std::string> joint_effort_interface_names_;
  std::vector<std::string> joint_wrench_interface_names_;

  std::vector<std::string> task_space_pose_interface_names_;
  std::vector<std::string> task_space_twist_interface_names_;
  std::vector<std::string> task_space_acceleration_interface_names_;
  std::vector<std::string> task_space_wrench_interface_names_;
  std::vector<std::string> task_space_wrench_derivative_interface_names_;

  // internal variables to store the semantic component commands
  std::unique_ptr<acg_semantic_components::JointPositionCommand> joint_position_command_;
  std::unique_ptr<acg_semantic_components::JointVelocityCommand> joint_velocity_command_;
  std::unique_ptr<acg_semantic_components::JointAccelerationCommand> joint_acceleration_command_;
  std::unique_ptr<acg_semantic_components::JointEffortCommand> joint_effort_command_;
  std::unique_ptr<acg_semantic_components::WrenchCommand> joint_wrench_command_;

  std::unique_ptr<acg_semantic_components::PoseCommand> task_space_pose_command_;
  std::unique_ptr<acg_semantic_components::TwistCommand> task_space_twist_command_;
  std::unique_ptr<acg_semantic_components::AccelCommand> task_space_acceleration_command_;
  std::unique_ptr<acg_semantic_components::WrenchCommand> task_space_wrench_command_;
  std::unique_ptr<acg_semantic_components::WrenchDerivativeCommand> task_space_wrench_derivative_command_;

  // internal flag to check if the object is configured
  bool configured_{ false };

  /**
   * @brief Assign the loaned command interfaces to this object. This function should be called once during the controller activation step. This
   * function is not real-time safe.
   *
   * This function assigns the loaned command interfaces to this object, based on the command interfaces specified in the configuration step.
   * This function should be called once during the controller activation step. This function is not real-time safe.
   * This function has protected visibility (instead of public) so that every subclass can expose it with a different signature if needed.
   *
   * @param[in] command_interfaces The vector of loaned command interfaces from which to assign the command interfaces.
   */
  void assign_loaned_command_interfaces(std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces);

  /**
   * @brief Returns whether the interface is configured. If not, logs an error message.
   *
   * @param[in] message The message to log in case the interface is not configured. The implementation provides a default message.
   *
   * @return true if the interface is configured, false otherwise.
   */
  bool is_configured_(const std::string& message = std::string()) const;
};

}  // namespace acg_hardware_interface_facade
