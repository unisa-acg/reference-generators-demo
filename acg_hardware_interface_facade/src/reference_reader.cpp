/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   reference_reader.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Jan 30, 2025
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include "acg_hardware_interface_facade/reference_reader.hpp"

namespace acg_hardware_interface_facade
{

void ReferenceReader::assign_command_interfaces(const std::vector<hardware_interface::CommandInterface>& external_command_interfaces,
                                                std::vector<double>& external_command_interfaces_data)
{
  if (!is_configured_())
  {
    return;
  }

  // The loaned command interfaces are generated only once
  if (has_generated_loaned_command_interfaces_)
  {
    return;
  }

  // Check if the number of external command interfaces is the same as the corresponding data vector
  if (external_command_interfaces.size() != external_command_interfaces_data.size())
  {
    RCLCPP_ERROR(rclcpp::get_logger("reference_reader"),
                 "The number of external command interfaces does not match the size of the corresponding data vector.");
    return;
  }

  // Create new command interfaces and bind them with the memory locations of the reference_interfaces vector
  command_interfaces_.reserve(external_command_interfaces.size());
  for (std::size_t i = 0; i < external_command_interfaces.size(); ++i)
  {
    command_interfaces_.emplace_back(external_command_interfaces[i].get_prefix_name(), external_command_interfaces[i].get_interface_name(),
                                     &external_command_interfaces_data[i]);
  }

  // Create the loaned command interfaces from the copied command interfaces
  loaned_command_interfaces_.reserve(external_command_interfaces.size());
  for (hardware_interface::CommandInterface& interface : command_interfaces_)
  {
    loaned_command_interfaces_.emplace_back(interface);
  }
  has_generated_loaned_command_interfaces_ = true;

  // Assign the loaned command interfaces to the SemanticComponentCommandInterface objects
  assign_loaned_command_interfaces(loaned_command_interfaces_);
}

void ReferenceReader::read_from_reference_interfaces(acg_control_msgs::msg::JointWrenchPoint& joint_wrench_msg) const
{
  if (!is_configured_())
  {
    return;
  }

  if (has_joint_position_interface_)
  {
    joint_position_command_->get_values_as_message(joint_wrench_msg.positions);
  }
  if (has_joint_velocity_interface_)
  {
    joint_velocity_command_->get_values_as_message(joint_wrench_msg.velocities);
  }
  if (has_joint_acceleration_interface_)
  {
    joint_acceleration_command_->get_values_as_message(joint_wrench_msg.accelerations);
  }
  if (has_joint_effort_interface_)
  {
    joint_effort_command_->get_values_as_message(joint_wrench_msg.effort);
  }
  if (has_joint_wrench_interface_)
  {
    joint_wrench_command_->get_values_as_message(joint_wrench_msg.wrench);
  }
}

void ReferenceReader::read_from_reference_interfaces(acg_control_msgs::msg::TaskSpacePoint& task_space_msg) const
{
  if (!is_configured_())
  {
    return;
  }

  if (has_task_space_pose_interface_)
  {
    task_space_pose_command_->get_values_as_message(task_space_msg.pose);
  }
  if (has_task_space_twist_interface_)
  {
    task_space_twist_command_->get_values_as_message(task_space_msg.twist);
  }
  if (has_task_space_acceleration_interface_)
  {
    task_space_acceleration_command_->get_values_as_message(task_space_msg.acceleration);
  }
  if (has_task_space_wrench_interface_)
  {
    task_space_wrench_command_->get_values_as_message(task_space_msg.wrench);
  }
  if (has_task_space_wrench_derivative_interface_)
  {
    task_space_wrench_derivative_command_->get_values_as_message(task_space_msg.wrench_derivative);
  }
}

std::vector<hardware_interface::CommandInterface> ReferenceReader::build_reference_interfaces(std::vector<double>& reference_interfaces)
{
  if (!is_configured_())
  {
    return {};
  }

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  std::vector<std::string> reference_names{ available_interfaces() };
  const std::size_t& size = reference_names.size();

  if (size != reference_interfaces.size())
  {
    RCLCPP_ERROR(rclcpp::get_logger("reference_reader"), "The number of reference interfaces does not match the size of the reference data vector.");
    return {};
  }

  // Create command interfaces
  command_interfaces.reserve(size);
  for (std::size_t i = 0; i < reference_names.size(); ++i)
  {
    // Take first part of the reference name, separated by a '/'
    std::string name = reference_names[i].substr(0, reference_names[i].find('/'));
    // Take second part of the reference name
    std::string interface = reference_names[i].substr(reference_names[i].find('/') + 1);
    command_interfaces.emplace_back(hardware_interface::CommandInterface(name, interface, &reference_interfaces[i]));
  }
  return command_interfaces;
}

bool ReferenceReader::is_configured_(const std::string& message) const
{
  return CommandInterfaceHandler::is_configured_(message.empty() ? "Reference reader not configured. Please configure the object before using it." :
                                                                   message);
}

void ReferenceReader::release_interfaces()
{
  if (!is_configured_())
  {
    return;
  }

  // Release the command interfaces
  CommandInterfaceHandler::release_interfaces();
  if (has_generated_loaned_command_interfaces_)
  {
    loaned_command_interfaces_.clear();
    command_interfaces_.clear();
    has_generated_loaned_command_interfaces_ = false;
  }
}

}  // namespace acg_hardware_interface_facade
