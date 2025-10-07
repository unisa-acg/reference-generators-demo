/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   command_interface_handler.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Jan 30, 2025
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <rclcpp/logging.hpp>
#include "acg_hardware_interface_facade/command_interface_handler.hpp"

namespace acg_hardware_interface_facade
{
bool CommandInterfaceHandler::configure_interfaces(const std::vector<std::string>& joint_names,
                                                   const std::vector<std::string>& joint_space_interfaces,
                                                   const std::vector<std::string>& task_space_interfaces,
                                                   const std::string& joint_space_controller_name, const std::string& task_space_controller_name,
                                                   const CommandInterfaceNamesOverrideConfig& override_config)
{
  if (configured_)
  {
    RCLCPP_WARN(rclcpp::get_logger("command_interface_handler"), "Command interface handler already configured.");
    return false;
  }

  num_joints_ = joint_names.size();
  joint_names_ = joint_names;
  joint_space_controller_name_ = joint_space_controller_name;
  task_space_controller_name_ = task_space_controller_name;

  // Check what interfaces are available for the joint space reference
  for (const std::string& interface : joint_space_interfaces)
  {
    if (interface == "position")
    {
      has_joint_position_interface_ = true;
    }
    else if (interface == "velocity")
    {
      has_joint_velocity_interface_ = true;
    }
    else if (interface == "acceleration")
    {
      has_joint_acceleration_interface_ = true;
    }
    else if (interface == "effort")
    {
      has_joint_effort_interface_ = true;
    }
    else if (interface == "wrench")
    {
      has_joint_wrench_interface_ = true;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("command_interface_handler"),
                   "The joint space interface '%s' is not supported. Supported interfaces are: position, velocity, acceleration, effort, wrench.",
                   interface.c_str());
      return false;
    }
  }

  // Check what interfaces are available for the task space reference
  for (const std::string& interface : task_space_interfaces)
  {
    if (interface == "pose")
    {
      has_task_space_pose_interface_ = true;
    }
    else if (interface == "twist")
    {
      has_task_space_twist_interface_ = true;
    }
    else if (interface == "acceleration")
    {
      has_task_space_acceleration_interface_ = true;
    }
    else if (interface == "wrench")
    {
      has_task_space_wrench_interface_ = true;
    }
    else if (interface == "wrench_derivative")
    {
      has_task_space_wrench_derivative_interface_ = true;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("command_interface_handler"),
                   "The task space interface '%s' is not supported. Supported interfaces are: pose, twist, acceleration, wrench, wrench_derivative.",
                   interface.c_str());
      return false;
    }
  }

  // Configure all the semantic component commands and save the interface names, according to the override configuration or the default one
  if (has_joint_position_interface_)
  {
    if (!override_config.joint_position_interface_names.empty())
    {
      if (override_config.joint_position_interface_names.size() != num_joints_)
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("command_interface_handler"),
            "The number of names in the override configuration (%ld) for the joint position interfaces does not match the number of joints (%ld).",
            override_config.joint_position_interface_names.size(), num_joints_);
        return false;
      }
      joint_position_interface_names_ = override_config.joint_position_interface_names;
    }
    else
    {
      for (const std::string& joint : joint_names_)
      {
        joint_position_interface_names_.emplace_back(joint_space_controller_name_ + "/" + joint + "/position");
      }
    }
    joint_position_command_ =
        std::make_unique<acg_semantic_components::JointPositionCommand>(joint_space_controller_name_, num_joints_, joint_position_interface_names_);
  }

  if (has_joint_velocity_interface_)
  {
    if (!override_config.joint_velocity_interface_names.empty())
    {
      if (override_config.joint_velocity_interface_names.size() != num_joints_)
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("command_interface_handler"),
            "The number of names in the override configuration (%ld) for the joint velocity interfaces does not match the number of joints (%ld).",
            override_config.joint_velocity_interface_names.size(), num_joints_);
        return false;
      }
      joint_velocity_interface_names_ = override_config.joint_velocity_interface_names;
    }
    else
    {
      for (const std::string& joint : joint_names_)
      {
        joint_velocity_interface_names_.emplace_back(joint_space_controller_name_ + "/" + joint + "/velocity");
      }
    }
    joint_velocity_command_ =
        std::make_unique<acg_semantic_components::JointVelocityCommand>(joint_space_controller_name_, num_joints_, joint_velocity_interface_names_);
  }

  if (has_joint_acceleration_interface_)
  {
    if (!override_config.joint_acceleration_interface_names.empty())
    {
      if (override_config.joint_acceleration_interface_names.size() != num_joints_)
      {
        RCLCPP_ERROR(rclcpp::get_logger("command_interface_handler"),
                     "The number of names in the override configuration (%ld) for the joint acceleration interfaces does not match the number of "
                     "joints (%ld).",
                     override_config.joint_acceleration_interface_names.size(), num_joints_);
        return false;
      }
      joint_acceleration_interface_names_ = override_config.joint_acceleration_interface_names;
    }
    else
    {
      for (const std::string& joint : joint_names_)
      {
        joint_acceleration_interface_names_.emplace_back(joint_space_controller_name_ + "/" + joint + "/acceleration");
      }
    }
    joint_acceleration_command_ = std::make_unique<acg_semantic_components::JointAccelerationCommand>(joint_space_controller_name_, num_joints_,
                                                                                                      joint_acceleration_interface_names_);
  }

  if (has_joint_effort_interface_)
  {
    if (!override_config.joint_effort_interface_names.empty())
    {
      if (override_config.joint_effort_interface_names.size() != num_joints_)
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("command_interface_handler"),
            "The number of names in the override configuration (%ld) for the joint effort interfaces does not match the number of joints (%ld).",
            override_config.joint_effort_interface_names.size(), num_joints_);
        return false;
      }
      joint_effort_interface_names_ = override_config.joint_effort_interface_names;
    }
    else
    {
      for (const std::string& joint : joint_names_)
      {
        joint_effort_interface_names_.emplace_back(joint_space_controller_name_ + "/" + joint + "/effort");
      }
    }
    joint_effort_command_ =
        std::make_unique<acg_semantic_components::JointEffortCommand>(joint_space_controller_name_, num_joints_, joint_effort_interface_names_);
  }

  if (has_joint_wrench_interface_)
  {
    if (!override_config.joint_wrench_interface_names.empty())
    {
      if (override_config.joint_wrench_interface_names.size() != acg_semantic_components::WrenchCommand::WRENCH_SIZE)
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("command_interface_handler"),
            "The number of names in the override configuration (%ld) for the joint wrench interfaces does not match the wrench message size (%hu).",
            override_config.joint_wrench_interface_names.size(), acg_semantic_components::WrenchCommand::WRENCH_SIZE);
        return false;
      }
      joint_wrench_command_ =
          std::make_unique<acg_semantic_components::WrenchCommand>(joint_space_controller_name_, override_config.joint_wrench_interface_names);
    }
    else
    {
      joint_wrench_command_ = std::make_unique<acg_semantic_components::WrenchCommand>(joint_space_controller_name_);
    }
    joint_wrench_interface_names_ = joint_wrench_command_->get_command_interface_names();
  }

  if (has_task_space_pose_interface_)
  {
    if (!override_config.task_space_pose_interface_names.empty())
    {
      if (override_config.task_space_pose_interface_names.size() != acg_semantic_components::PoseCommand::POSE_SIZE)
      {
        RCLCPP_ERROR(rclcpp::get_logger("command_interface_handler"),
                     "The number of names in the override configuration (%ld) for the task space pose "
                     "interfaces does not match the pose message size (%hu).",
                     override_config.task_space_pose_interface_names.size(), acg_semantic_components::PoseCommand::POSE_SIZE);
        return false;
      }
      task_space_pose_command_ =
          std::make_unique<acg_semantic_components::PoseCommand>(task_space_controller_name_, override_config.task_space_pose_interface_names);
    }
    else
    {
      task_space_pose_command_ = std::make_unique<acg_semantic_components::PoseCommand>(task_space_controller_name_);
    }
    task_space_pose_interface_names_ = task_space_pose_command_->get_command_interface_names();
  }

  if (has_task_space_twist_interface_)
  {
    if (!override_config.task_space_twist_interface_names.empty())
    {
      if (override_config.task_space_twist_interface_names.size() != acg_semantic_components::TwistCommand::TWIST_SIZE)
      {
        RCLCPP_ERROR(rclcpp::get_logger("command_interface_handler"),
                     "The number of names in the override configuration (%ld) for the task space twist "
                     "interfaces does not match the twist message size (%hu).",
                     override_config.task_space_twist_interface_names.size(), acg_semantic_components::TwistCommand::TWIST_SIZE);
        return false;
      }
      task_space_twist_command_ =
          std::make_unique<acg_semantic_components::TwistCommand>(task_space_controller_name_, override_config.task_space_twist_interface_names);
    }
    else
    {
      task_space_twist_command_ = std::make_unique<acg_semantic_components::TwistCommand>(task_space_controller_name_);
    }
    task_space_twist_interface_names_ = task_space_twist_command_->get_command_interface_names();
  }

  if (has_task_space_acceleration_interface_)
  {
    if (!override_config.task_space_acceleration_interface_names.empty())
    {
      if (override_config.task_space_acceleration_interface_names.size() != acg_semantic_components::AccelCommand::ACCEL_SIZE)
      {
        RCLCPP_ERROR(rclcpp::get_logger("command_interface_handler"),
                     "The number of names in the override configuration (%ld) for the task space "
                     "acceleration interfaces does not match the acceleration message size (%hu).",
                     override_config.task_space_acceleration_interface_names.size(), acg_semantic_components::AccelCommand::ACCEL_SIZE);
        return false;
      }
      task_space_acceleration_command_ = std::make_unique<acg_semantic_components::AccelCommand>(
          task_space_controller_name_, override_config.task_space_acceleration_interface_names);
    }
    else
    {
      task_space_acceleration_command_ = std::make_unique<acg_semantic_components::AccelCommand>(task_space_controller_name_);
    }
    task_space_acceleration_interface_names_ = task_space_acceleration_command_->get_command_interface_names();
  }

  if (has_task_space_wrench_interface_)
  {
    if (!override_config.task_space_wrench_interface_names.empty())
    {
      if (override_config.task_space_wrench_interface_names.size() != acg_semantic_components::WrenchCommand::WRENCH_SIZE)
      {
        RCLCPP_ERROR(rclcpp::get_logger("command_interface_handler"),
                     "The number of names in the override configuration (%ld) for the task space wrench "
                     "interfaces does not match the wrench message size (%hu).",
                     override_config.task_space_wrench_interface_names.size(), acg_semantic_components::WrenchCommand::WRENCH_SIZE);
        return false;
      }
      task_space_wrench_command_ =
          std::make_unique<acg_semantic_components::WrenchCommand>(task_space_controller_name_, override_config.task_space_wrench_interface_names);
    }
    else
    {
      task_space_wrench_command_ = std::make_unique<acg_semantic_components::WrenchCommand>(task_space_controller_name_);
    }
    task_space_wrench_interface_names_ = task_space_wrench_command_->get_command_interface_names();
  }

  if (has_task_space_wrench_derivative_interface_)
  {
    if (!override_config.task_space_wrench_derivative_interface_names.empty())
    {
      if (override_config.task_space_wrench_derivative_interface_names.size() != acg_semantic_components::WrenchDerivativeCommand::WRENCH_SIZE)
      {
        RCLCPP_ERROR(rclcpp::get_logger("command_interface_handler"),
                     "The number of names in the override configuration (%ld) for the task space wrench "
                     "derivative interfaces does not match the task space message size (%hu).",
                     override_config.task_space_wrench_derivative_interface_names.size(),
                     acg_semantic_components::WrenchDerivativeCommand::WRENCH_SIZE);
        return false;
      }
      task_space_wrench_derivative_command_ = std::make_unique<acg_semantic_components::WrenchDerivativeCommand>(
          task_space_controller_name_, override_config.task_space_wrench_derivative_interface_names);
    }
    else
    {
      task_space_wrench_derivative_command_ = std::make_unique<acg_semantic_components::WrenchDerivativeCommand>(task_space_controller_name_);
    }
    task_space_wrench_derivative_interface_names_ = task_space_wrench_derivative_command_->get_command_interface_names();
  }

  // Set the object as configured
  configured_ = true;

  return true;
}

std::vector<std::string> CommandInterfaceHandler::available_interfaces() const
{
  // Check if the object is configured
  if (!is_configured_())
  {
    return {};
  }

  std::vector<std::string> interfaces_config_names;

  // Retrieve the joint space interfaces
  if (!joint_space_controller_name_.empty())
  {
    if (has_joint_position_interface_)
    {
      interfaces_config_names.insert(interfaces_config_names.end(), joint_position_interface_names_.begin(), joint_position_interface_names_.end());
    }
    if (has_joint_velocity_interface_)
    {
      interfaces_config_names.insert(interfaces_config_names.end(), joint_velocity_interface_names_.begin(), joint_velocity_interface_names_.end());
    }
    if (has_joint_acceleration_interface_)
    {
      interfaces_config_names.insert(interfaces_config_names.end(), joint_acceleration_interface_names_.begin(),
                                     joint_acceleration_interface_names_.end());
    }
    if (has_joint_effort_interface_)
    {
      interfaces_config_names.insert(interfaces_config_names.end(), joint_effort_interface_names_.begin(), joint_effort_interface_names_.end());
    }
    if (has_joint_wrench_interface_)
    {
      interfaces_config_names.insert(interfaces_config_names.end(), joint_wrench_interface_names_.begin(), joint_wrench_interface_names_.end());
    }
  }

  // Retrieve the task space interfaces
  if (!task_space_controller_name_.empty())
  {
    if (has_task_space_pose_interface_)
    {
      interfaces_config_names.insert(interfaces_config_names.end(), task_space_pose_interface_names_.begin(), task_space_pose_interface_names_.end());
    }
    if (has_task_space_twist_interface_)
    {
      interfaces_config_names.insert(interfaces_config_names.end(), task_space_twist_interface_names_.begin(),
                                     task_space_twist_interface_names_.end());
    }
    if (has_task_space_acceleration_interface_)
    {
      interfaces_config_names.insert(interfaces_config_names.end(), task_space_acceleration_interface_names_.begin(),
                                     task_space_acceleration_interface_names_.end());
    }
    if (has_task_space_wrench_interface_)
    {
      interfaces_config_names.insert(interfaces_config_names.end(), task_space_wrench_interface_names_.begin(),
                                     task_space_wrench_interface_names_.end());
    }
    if (has_task_space_wrench_derivative_interface_)
    {
      interfaces_config_names.insert(interfaces_config_names.end(), task_space_wrench_derivative_interface_names_.begin(),
                                     task_space_wrench_derivative_interface_names_.end());
    }
  }

  return interfaces_config_names;
}

bool CommandInterfaceHandler::has_joint_position_interface() const
{
  return has_joint_position_interface_;
}

bool CommandInterfaceHandler::has_joint_velocity_interface() const
{
  return has_joint_velocity_interface_;
}

bool CommandInterfaceHandler::has_joint_acceleration_interface() const
{
  return has_joint_acceleration_interface_;
}

bool CommandInterfaceHandler::has_joint_effort_interface() const
{
  return has_joint_effort_interface_;
}

bool CommandInterfaceHandler::has_joint_wrench_interface() const
{
  return has_joint_wrench_interface_;
}

bool CommandInterfaceHandler::has_task_space_pose_interface() const
{
  return has_task_space_pose_interface_;
}

bool CommandInterfaceHandler::has_task_space_twist_interface() const
{
  return has_task_space_twist_interface_;
}

bool CommandInterfaceHandler::has_task_space_acceleration_interface() const
{
  return has_task_space_acceleration_interface_;
}

bool CommandInterfaceHandler::has_task_space_wrench_interface() const
{
  return has_task_space_wrench_interface_;
}

bool CommandInterfaceHandler::has_task_space_wrench_derivative_interface() const
{
  return has_task_space_wrench_derivative_interface_;
}

void CommandInterfaceHandler::release_interfaces()
{
  if (!is_configured_())
  {
    return;
  }
  if (has_joint_position_interface_)
  {
    joint_position_command_->release_interfaces();
  }
  if (has_joint_velocity_interface_)
  {
    joint_velocity_command_->release_interfaces();
  }
  if (has_joint_acceleration_interface_)
  {
    joint_acceleration_command_->release_interfaces();
  }
  if (has_joint_effort_interface_)
  {
    joint_effort_command_->release_interfaces();
  }
  if (has_joint_wrench_interface_)
  {
    joint_wrench_command_->release_interfaces();
  }
  if (has_task_space_pose_interface_)
  {
    task_space_pose_command_->release_interfaces();
  }
  if (has_task_space_twist_interface_)
  {
    task_space_twist_command_->release_interfaces();
  }
  if (has_task_space_acceleration_interface_)
  {
    task_space_acceleration_command_->release_interfaces();
  }
  if (has_task_space_wrench_interface_)
  {
    task_space_wrench_command_->release_interfaces();
  }
  if (has_task_space_wrench_derivative_interface_)
  {
    task_space_wrench_derivative_command_->release_interfaces();
  }
}

void CommandInterfaceHandler::assign_loaned_command_interfaces(std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces)
{
  if (!is_configured_())
  {
    return;
  }

  if (has_joint_position_interface_)
  {
    joint_position_command_->assign_loaned_command_interfaces(command_interfaces);
  }
  if (has_joint_velocity_interface_)
  {
    joint_velocity_command_->assign_loaned_command_interfaces(command_interfaces);
  }
  if (has_joint_acceleration_interface_)
  {
    joint_acceleration_command_->assign_loaned_command_interfaces(command_interfaces);
  }
  if (has_joint_effort_interface_)
  {
    joint_effort_command_->assign_loaned_command_interfaces(command_interfaces);
  }
  if (has_joint_wrench_interface_)
  {
    joint_wrench_command_->assign_loaned_command_interfaces(command_interfaces);
  }

  if (has_task_space_pose_interface_)
  {
    task_space_pose_command_->assign_loaned_command_interfaces(command_interfaces);
  }
  if (has_task_space_twist_interface_)
  {
    task_space_twist_command_->assign_loaned_command_interfaces(command_interfaces);
  }
  if (has_task_space_acceleration_interface_)
  {
    task_space_acceleration_command_->assign_loaned_command_interfaces(command_interfaces);
  }
  if (has_task_space_wrench_interface_)
  {
    task_space_wrench_command_->assign_loaned_command_interfaces(command_interfaces);
  }
  if (has_task_space_wrench_derivative_interface_)
  {
    task_space_wrench_derivative_command_->assign_loaned_command_interfaces(command_interfaces);
  }
}

bool CommandInterfaceHandler::is_configured_(const std::string& message) const
{
  RCLCPP_ERROR_EXPRESSION(rclcpp::get_logger("command_interface_handler"), !configured_, "%s",
                          message.empty() ? "Command interface handler not configured. Please configure the object before using it." :
                                            message.c_str());
  return configured_;
}

}  // namespace acg_hardware_interface_facade
