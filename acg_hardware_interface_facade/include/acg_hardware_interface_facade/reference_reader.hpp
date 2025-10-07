/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   reference_reader.hpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Jan 30, 2025
 *
 * This module contains the ReferenceReader class, which is used to
 * read joint space and task space references from the reference
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
 * @brief Class that handles the reading of the reference interfaces, based on the reference interfaces specified in the configuration step.
 */
class ReferenceReader : public CommandInterfaceHandler
{
public:
  /**
   * @brief Construct a new reference reader object. This function is not real-time safe.
   */
  ReferenceReader() = default;

  /**
   * @brief Destruct this object. This function is not real-time safe.
   */
  ~ReferenceReader() = default;

  /**
   * @brief Assign the command interfaces to the reference reader. This function should be called once during the controller activation step.
   *
   * This function assigns the command interfaces to the reference reader, based on the command interfaces specified in the configuration step.
   * This function internally creates a new vector of command interfaces owned by the class based on the provided input.
   * The internal command interfaces are defined on the same memory locations of the input command interfaces, which are
   * represented by the \p external_command_interfaces_data vector.
   * This function is not real-time safe.
   * For more details on this function's design and implementation, please refer to \c reference_reader_implementation_notes.md file.
   *
   * @param[in] external_command_interfaces The vector of external command interfaces. The method uses the metadata of the command interfaces passed
   * as input to create new command interfaces owned by the class.
   * @param[in] external_command_interfaces_data The vector of doubles associated with the command interfaces passed as input. It must be the same
   * vector that was used to construct the \p external_command_interfaces.
   */
  void assign_command_interfaces(const std::vector<hardware_interface::CommandInterface>& external_command_interfaces,
                                 std::vector<double>& external_command_interfaces_data);

  /**
   * @brief Read the joint space reference from the reference interfaces.
   *
   * This function reads the joint space references specified via the \c configure_interfaces function. This function is real-time safe.
   *
   * @param[out] joint_wrench_msg The variable where the joint space reference will be stored.
   */
  void read_from_reference_interfaces(acg_control_msgs::msg::JointWrenchPoint& joint_wrench_msg) const;

  /**
   * @brief Read the task space reference from the reference interfaces.
   *
   * This function reads the task space references specified via the \c configure_interfaces function. This function is real-time safe.
   *
   * @param[out] task_space_msg The variable where the task space reference will be stored.
   */
  void read_from_reference_interfaces(acg_control_msgs::msg::TaskSpacePoint& task_space_msg) const;

  /**
   * @brief Build the reference interfaces given the vector of doubles storing the data of the reference interfaces.
   *
   * This function is called in the \c on_export_reference_interfaces method of a chainable controller. If the size of the \p reference_interfaces
   * vector does not match the size of the reference interfaces, an error message is logged and an empty vector is returned. This function is not
   * real-time safe.
   *
   * @param[in] reference_interfaces The vector of doubles storing the data of the reference interfaces.
   * @return A vector of command interfaces that can be used to read the reference interfaces.
   */
  std::vector<hardware_interface::CommandInterface> build_reference_interfaces(std::vector<double>& reference_interfaces);

  /**
   * @brief Refer to the superclass documentation
   */
  void release_interfaces() override;

private:
  bool has_generated_loaned_command_interfaces_{ false };
  std::vector<hardware_interface::CommandInterface> command_interfaces_;
  std::vector<hardware_interface::LoanedCommandInterface> loaned_command_interfaces_;

  /**
   * @brief Refer to the superclass documentation
   */
  bool is_configured_(const std::string& message = std::string()) const;
};

}  // namespace acg_hardware_interface_facade
