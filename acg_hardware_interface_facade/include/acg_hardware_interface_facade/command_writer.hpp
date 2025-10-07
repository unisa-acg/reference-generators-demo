/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   command_writer.hpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Jan 30, 2025
 *
 * This module contains the CommandWriter class, which is used to
 * write joint space and task space commands to the command
 * interfaces.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <vector>
#include <string>

#include <hardware_interface/loaned_state_interface.hpp>

#include <acg_control_msgs/msg/joint_wrench_point.hpp>
#include <acg_control_msgs/msg/task_space_point.hpp>

#include "acg_hardware_interface_facade/command_interface_handler.hpp"

namespace acg_hardware_interface_facade
{

/**
 * @brief Class that handles the writing on the command interfaces, based on the command interfaces specified in the configuration step.
 */
class CommandWriter : public CommandInterfaceHandler
{
public:
  /**
   * @brief Construct a new command writer object. This function is not real-time safe.
   */
  CommandWriter() = default;

  /**
   * @brief Destruct this object. This function is not real-time safe.
   */
  ~CommandWriter() = default;

  /**
   * @brief Write a joint space message to the command interfaces.
   *
   * This function writes the fields of the \c acg_control_msgs::msg::JointWrenchPoint to the command interfaces, according to the command interfaces
   * specified via the \c configure_interfaces method. This function is real-time safe.
   *
   * @param[in] joint_wrench_msg The joint space message to write.
   */
  void write_to_command_interfaces(const acg_control_msgs::msg::JointWrenchPoint& joint_wrench_msg);

  /**
   * @brief Write the task space reference to the command interfaces.
   *
   * This function writes the fields of the \c acg_control_msgs::msg::TaskSpacePoint to the command interfaces, according to the command interfaces
   * specified via the \c configure_interfaces method. This function is real-time safe.
   *
   * @param[in] task_space_msg The task space message to write.
   */
  void write_to_command_interfaces(const acg_control_msgs::msg::TaskSpacePoint& task_space_msg);

  /**
   * @brief Assign the loaned command interfaces to the command writer. This function should be called once during the controller activation step.
   *
   * This function assigns the loaned command interfaces to the command writer, based on the command interfaces specified in the configuration step.
   * This function should be called once during the controller activation step. This function is not real-time safe.
   * This is a wrapper for the protected method \c CommandInterfaceHandler::assign_loaned_command_interfaces.
   *
   * @param[in] command_interfaces The vector of loaned command interfaces to save and handle.
   */
  void assign_loaned_command_interfaces(std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces);

private:
  /**
   * @brief Refer to the superclass documentation
   */
  bool is_configured_(const std::string& message = std::string()) const;
};

}  // namespace acg_hardware_interface_facade
