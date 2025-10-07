/*
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   twist_command.hpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 17, 2025
 *
 * This class implements a twist command semantic component.
 *
 * -------------------------------------------------------------------
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "acg_semantic_components/semantic_component_command_interface.hpp"

namespace acg_semantic_components
{

class TwistCommand : public semantic_components::SemanticComponentCommandInterface<geometry_msgs::msg::Twist>
{
private:
  std::vector<std::string> generate_interface_names_(const std::string& name, const std::vector<std::string>& interface_names)
  {
    if (!interface_names.empty())
    {
      if (interface_names.size() != TWIST_SIZE)
      {
        throw std::invalid_argument("TwistCommand: interface_names size (" + std::to_string(interface_names.size()) + ") must be equal to " +
                                    std::to_string(TWIST_SIZE));
      }
      return interface_names;
    }
    else
    {
      // Return standard interface names
      return { { name + "/linear_velocity.x" },  { name + "/linear_velocity.y" },  { name + "/linear_velocity.z" },
               { name + "/angular_velocity.x" }, { name + "/angular_velocity.y" }, { name + "/angular_velocity.z" } };
    }
  }

public:
  static constexpr unsigned short int TWIST_SIZE{ 6 };

  TwistCommand(const std::string& name, const std::vector<std::string>& interface_names = {})
    : SemanticComponentCommandInterface(name, generate_interface_names_(name, interface_names))
  {}

  ~TwistCommand() = default;

  bool set_values_from_message(const geometry_msgs::msg::Twist& twist) override
  {
    if (command_interfaces_.size() != TWIST_SIZE)
    {
      return false;
    }
    command_interfaces_[0].get().set_value(twist.linear.x);
    command_interfaces_[1].get().set_value(twist.linear.y);
    command_interfaces_[2].get().set_value(twist.linear.z);
    command_interfaces_[3].get().set_value(twist.angular.x);
    command_interfaces_[4].get().set_value(twist.angular.y);
    command_interfaces_[5].get().set_value(twist.angular.z);

    return true;
  }

  bool get_values_as_message(geometry_msgs::msg::Twist& twist) override
  {
    if (command_interfaces_.size() != TWIST_SIZE)
    {
      return false;
    }
    twist.linear.x = command_interfaces_[0].get().get_value();
    twist.linear.y = command_interfaces_[1].get().get_value();
    twist.linear.z = command_interfaces_[2].get().get_value();
    twist.angular.x = command_interfaces_[3].get().get_value();
    twist.angular.y = command_interfaces_[4].get().get_value();
    twist.angular.z = command_interfaces_[5].get().get_value();

    return true;
  }
};

}  // namespace acg_semantic_components
