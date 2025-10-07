/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   command_writer.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Jan 30, 2025
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include "acg_hardware_interface_facade/command_writer.hpp"

namespace acg_hardware_interface_facade
{

void CommandWriter::write_to_command_interfaces(const acg_control_msgs::msg::JointWrenchPoint& joint_wrench_msg)
{
  if (!is_configured_())
  {
    return;
  }
  // Writing the reference to the command interfaces using the semantic command components
  if (has_joint_position_interface_)
  {
    joint_position_command_->set_values_from_message(joint_wrench_msg.positions);
  }
  if (has_joint_velocity_interface_)
  {
    joint_velocity_command_->set_values_from_message(joint_wrench_msg.velocities);
  }
  if (has_joint_acceleration_interface_)
  {
    joint_acceleration_command_->set_values_from_message(joint_wrench_msg.accelerations);
  }
  if (has_joint_effort_interface_)
  {
    joint_effort_command_->set_values_from_message(joint_wrench_msg.effort);
  }
  if (has_joint_wrench_interface_)
  {
    joint_wrench_command_->set_values_from_message(joint_wrench_msg.wrench);
  }
}

void CommandWriter::write_to_command_interfaces(const acg_control_msgs::msg::TaskSpacePoint& task_space_msg)
{
  if (!is_configured_())
  {
    return;
  }

  // Writing the reference to the command interfaces using the semantic command components
  if (has_task_space_pose_interface_)
  {
    task_space_pose_command_->set_values_from_message(task_space_msg.pose);
  }
  if (has_task_space_twist_interface_)
  {
    task_space_twist_command_->set_values_from_message(task_space_msg.twist);
  }
  if (has_task_space_acceleration_interface_)
  {
    task_space_acceleration_command_->set_values_from_message(task_space_msg.acceleration);
  }
  if (has_task_space_wrench_interface_)
  {
    task_space_wrench_command_->set_values_from_message(task_space_msg.wrench);
  }
  if (has_task_space_wrench_derivative_interface_)
  {
    task_space_wrench_derivative_command_->set_values_from_message(task_space_msg.wrench_derivative);
  }
}

void CommandWriter::assign_loaned_command_interfaces(std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces)
{
  if (!is_configured_())
  {
    return;
  }

  CommandInterfaceHandler::assign_loaned_command_interfaces(command_interfaces);
}

bool CommandWriter::is_configured_(const std::string& message) const
{
  return CommandInterfaceHandler::is_configured_(message.empty() ? "Command writer not configured. Please configure the object before using it." :
                                                                   message);
}

}  // namespace acg_hardware_interface_facade
