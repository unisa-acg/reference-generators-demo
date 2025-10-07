/*
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   accel_command.hpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 17, 2025
 *
 * This class implements a task space acceleration command
 * semantic component.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include "acg_semantic_components/semantic_component_command_interface.hpp"

namespace acg_semantic_components
{

class AccelCommand : public semantic_components::SemanticComponentCommandInterface<geometry_msgs::msg::Accel>
{
private:
  std::vector<std::string> generate_interface_names_(const std::string& name, const std::vector<std::string>& interface_names)
  {
    if (!interface_names.empty())
    {
      if (interface_names.size() != ACCEL_SIZE)
      {
        throw std::invalid_argument("AccelCommand: interface_names size (" + std::to_string(interface_names.size()) + ") must be equal to " +
                                    std::to_string(ACCEL_SIZE));
      }
      return interface_names;
    }
    else
    {
      // Return standard interface names
      return { { name + "/linear_acceleration.x" },  { name + "/linear_acceleration.y" },  { name + "/linear_acceleration.z" },
               { name + "/angular_acceleration.x" }, { name + "/angular_acceleration.y" }, { name + "/angular_acceleration.z" } };
    }
  }

public:
  static constexpr unsigned short int ACCEL_SIZE{ 6 };

  AccelCommand(const std::string& name, const std::vector<std::string>& interface_names = {})
    : SemanticComponentCommandInterface(name, generate_interface_names_(name, interface_names))
  {}

  ~AccelCommand() = default;

  bool set_values_from_message(const geometry_msgs::msg::Accel& accel) override
  {
    if (command_interfaces_.size() != ACCEL_SIZE)
    {
      return false;
    }
    command_interfaces_[0].get().set_value(accel.linear.x);
    command_interfaces_[1].get().set_value(accel.linear.y);
    command_interfaces_[2].get().set_value(accel.linear.z);
    command_interfaces_[3].get().set_value(accel.angular.x);
    command_interfaces_[4].get().set_value(accel.angular.y);
    command_interfaces_[5].get().set_value(accel.angular.z);

    return true;
  }

  bool get_values_as_message(geometry_msgs::msg::Accel& accel) override
  {
    if (command_interfaces_.size() != ACCEL_SIZE)
    {
      return false;
    }
    accel.linear.x = command_interfaces_[0].get().get_value();
    accel.linear.y = command_interfaces_[1].get().get_value();
    accel.linear.z = command_interfaces_[2].get().get_value();
    accel.angular.x = command_interfaces_[3].get().get_value();
    accel.angular.y = command_interfaces_[4].get().get_value();
    accel.angular.z = command_interfaces_[5].get().get_value();

    return true;
  }
};

}  // namespace acg_semantic_components
