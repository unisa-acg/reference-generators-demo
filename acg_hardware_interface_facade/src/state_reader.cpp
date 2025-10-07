/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   state_reader.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Jan 30, 2025
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include "acg_hardware_interface_facade/state_reader.hpp"

namespace acg_hardware_interface_facade
{

bool StateReader::configure_state_interfaces(const std::vector<std::string>& state_interfaces, const std::vector<std::string>& joint_names,
                                             const std::string& robot_name, const StateInterfaceNamesOverrideConfig& override_config)
{
  if (configured_)
  {
    RCLCPP_WARN(rclcpp::get_logger("state_reader"), "State reader already configured.");
    return true;
  }

  joint_names_ = joint_names;
  robot_name_ = robot_name;
  num_joints_ = joint_names.size();

  for (const std::string& interface : state_interfaces)
  {
    if (interface == "position")
    {
      has_joint_position_state_interface_ = true;
    }
    else if (interface == "velocity")
    {
      has_joint_velocity_state_interface_ = true;
    }
    else if (interface == "acceleration")
    {
      has_joint_acceleration_state_interface_ = true;
    }
    else if (interface == "effort")
    {
      has_joint_effort_state_interface_ = true;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("state_reader"),
                   "The state interface %s is not supported. Supported interfaces are: position, velocity, acceleration, effort.", interface.c_str());
      return false;
    }
  }

  // Configuring the semantic components and saving the names of the state interfaces, using the override configuration if provided
  if (has_joint_position_state_interface_)
  {
    if (!override_config.position_state_interfaces.empty())
    {
      if (override_config.position_state_interfaces.size() != num_joints_)
      {
        RCLCPP_ERROR(rclcpp::get_logger("state_reader"),
                     "The number of names in the override configuration for the position state interfaces does not match the number of joints.");
        return false;
      }
      position_interfaces_names_ = override_config.position_state_interfaces;
    }
    else
    {
      for (const std::string& joint : joint_names_)
      {
        position_interfaces_names_.push_back(robot_name_ + "/" + joint + "/position");
      }
    }
    position_encoder_ = std::make_unique<acg_semantic_components::PositionEncoder>(robot_name_, num_joints_, position_interfaces_names_);
  }

  if (has_joint_velocity_state_interface_)
  {
    if (!override_config.velocity_state_interfaces.empty())
    {
      if (override_config.velocity_state_interfaces.size() != num_joints_)
      {
        RCLCPP_ERROR(rclcpp::get_logger("state_reader"),
                     "The number of names in the override configuration for the velocity state interfaces does not match the number of joints.");
        return false;
      }
      velocity_interfaces_names_ = override_config.velocity_state_interfaces;
    }
    else
    {
      for (const std::string& joint : joint_names_)
      {
        velocity_interfaces_names_.push_back(robot_name_ + "/" + joint + "/velocity");
      }
    }
    velocity_encoder_ = std::make_unique<acg_semantic_components::VelocityEncoder>(robot_name_, num_joints_, velocity_interfaces_names_);
  }

  if (has_joint_acceleration_state_interface_)
  {
    if (!override_config.acceleration_state_interfaces.empty())
    {
      if (override_config.acceleration_state_interfaces.size() != num_joints_)
      {
        RCLCPP_ERROR(rclcpp::get_logger("state_reader"),
                     "The number of names in the override configuration for the acceleration state interfaces does not match the number of joints.");
        return false;
      }
      acceleration_interfaces_names_ = override_config.acceleration_state_interfaces;
    }
    else
    {
      for (const std::string& joint : joint_names_)
      {
        acceleration_interfaces_names_.push_back(robot_name_ + "/" + joint + "/acceleration");
      }
    }
    accelerometer_ = std::make_unique<acg_semantic_components::Accelerometer>(robot_name_, num_joints_, acceleration_interfaces_names_);
  }

  if (has_joint_effort_state_interface_)
  {
    if (!override_config.effort_state_interfaces.empty())
    {
      if (override_config.effort_state_interfaces.size() != num_joints_)
      {
        RCLCPP_ERROR(rclcpp::get_logger("state_reader"),
                     "The number of names in the override configuration for the effort state interfaces does not match the number of joints.");
        return false;
      }
      effort_interfaces_names_ = override_config.effort_state_interfaces;
    }
    else
    {
      for (const std::string& joint : joint_names_)
      {
        effort_interfaces_names_.push_back(robot_name_ + "/" + joint + "/effort");
      }
    }
    effort_sensor_ = std::make_unique<acg_semantic_components::EffortSensor>(robot_name_, num_joints_, effort_interfaces_names_);
  }

  configured_ = true;
  return true;
}

std::vector<std::string> StateReader::available_state_interfaces() const
{
  if (!is_configured_())
  {
    return {};
  }

  std::vector<std::string> state_interfaces_config_names;

  if (has_joint_position_state_interface_)
  {
    state_interfaces_config_names.insert(state_interfaces_config_names.end(), position_interfaces_names_.begin(), position_interfaces_names_.end());
  }
  if (has_joint_velocity_state_interface_)
  {
    state_interfaces_config_names.insert(state_interfaces_config_names.end(), velocity_interfaces_names_.begin(), velocity_interfaces_names_.end());
  }
  if (has_joint_acceleration_state_interface_)
  {
    state_interfaces_config_names.insert(state_interfaces_config_names.end(), acceleration_interfaces_names_.begin(),
                                         acceleration_interfaces_names_.end());
  }
  if (has_joint_effort_state_interface_)
  {
    state_interfaces_config_names.insert(state_interfaces_config_names.end(), effort_interfaces_names_.begin(), effort_interfaces_names_.end());
  }

  return state_interfaces_config_names;
}

void StateReader::read_state_interfaces(RobotJointState& state) const
{
  if (!is_configured_())
  {
    return;
  }

  if (has_joint_position_state_interface_ && !position_encoder_->get_values_as_message(state.positions))
  {
    RCLCPP_ERROR(rclcpp::get_logger("state_reader"), "Error reading values from position encoder");
  }
  if (has_joint_velocity_state_interface_ && !velocity_encoder_->get_values_as_message(state.velocities))
  {
    RCLCPP_ERROR(rclcpp::get_logger("state_reader"), "Error reading values from velocity encoder");
  }
  if (has_joint_acceleration_state_interface_ && !accelerometer_->get_values_as_message(state.accelerations))
  {
    RCLCPP_ERROR(rclcpp::get_logger("state_reader"), "Error reading values from accelerometer");
  }
  if (has_joint_effort_state_interface_ && !effort_sensor_->get_values_as_message(state.efforts))
  {
    RCLCPP_ERROR(rclcpp::get_logger("state_reader"), "Error reading values from effort sensor");
  }
}

bool StateReader::has_joint_position_state_interface() const
{
  return has_joint_position_state_interface_;
}

bool StateReader::has_joint_velocity_state_interface() const
{
  return has_joint_velocity_state_interface_;
}

bool StateReader::has_joint_acceleration_state_interface() const
{
  return has_joint_acceleration_state_interface_;
}

bool StateReader::has_joint_effort_state_interface() const
{
  return has_joint_effort_state_interface_;
}

bool StateReader::assign_loaned_state_interfaces(std::vector<hardware_interface::LoanedStateInterface>& state_interfaces)
{
  if (!is_configured_())
  {
    return false;
  }

  bool all_true = true;

  if (has_joint_position_state_interface_)
  {
    all_true &= position_encoder_->assign_loaned_state_interfaces(state_interfaces);
  }
  if (has_joint_velocity_state_interface_)
  {
    all_true &= velocity_encoder_->assign_loaned_state_interfaces(state_interfaces);
  }
  if (has_joint_acceleration_state_interface_)
  {
    all_true &= accelerometer_->assign_loaned_state_interfaces(state_interfaces);
  }
  if (has_joint_effort_state_interface_)
  {
    all_true &= effort_sensor_->assign_loaned_state_interfaces(state_interfaces);
  }

  return all_true;
}

void StateReader::release_interfaces()
{
  if (!is_configured_())
  {
    return;
  }

  // Release the interfaces
  if (has_joint_position_state_interface_)
  {
    position_encoder_->release_interfaces();
  }
  if (has_joint_velocity_state_interface_)
  {
    velocity_encoder_->release_interfaces();
  }
  if (has_joint_acceleration_state_interface_)
  {
    accelerometer_->release_interfaces();
  }
  if (has_joint_effort_state_interface_)
  {
    effort_sensor_->release_interfaces();
  }
}

bool StateReader::is_configured_() const
{
  RCLCPP_ERROR_EXPRESSION(rclcpp::get_logger("state_reader"), !configured_,
                          "State reader not configured. Please configure the object before using this class.");
  return configured_;
}

}  // namespace acg_hardware_interface_facade
