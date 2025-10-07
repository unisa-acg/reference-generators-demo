/*
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   wrench_command.hpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 17, 2025
 *
 * This class implements a wrench command semantic component.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include "acg_semantic_components/semantic_component_command_interface.hpp"

namespace acg_semantic_components
{

class WrenchCommand : public semantic_components::SemanticComponentCommandInterface<geometry_msgs::msg::Wrench>
{
private:
  std::vector<std::string> generate_interface_names_(const std::string& name, const std::vector<std::string>& interface_names)
  {
    if (!interface_names.empty())
    {
      if (interface_names.size() != WRENCH_SIZE)
      {
        throw std::invalid_argument("WrenchCommand: interface_names size (" + std::to_string(interface_names.size()) + ") must be equal to " +
                                    std::to_string(WRENCH_SIZE));
      }
      return interface_names;
    }
    else
    {
      // Return standard interface names
      return { { name + "/force.x" },  { name + "/force.y" },  { name + "/force.z" },
               { name + "/torque.x" }, { name + "/torque.y" }, { name + "/torque.z" } };
    }
  }

public:
  static constexpr unsigned short int WRENCH_SIZE{ 6 };

  WrenchCommand(const std::string& name, const std::vector<std::string>& interface_names = {})
    : SemanticComponentCommandInterface(name, generate_interface_names_(name, interface_names))
  {}

  ~WrenchCommand() = default;

  bool set_values_from_message(const geometry_msgs::msg::Wrench& values) override
  {
    if (command_interfaces_.size() != WRENCH_SIZE)
    {
      return false;
    }
    command_interfaces_[0].get().set_value(values.force.x);
    command_interfaces_[1].get().set_value(values.force.y);
    command_interfaces_[2].get().set_value(values.force.z);
    command_interfaces_[3].get().set_value(values.torque.x);
    command_interfaces_[4].get().set_value(values.torque.y);
    command_interfaces_[5].get().set_value(values.torque.z);

    return true;
  }

  bool get_values_as_message(geometry_msgs::msg::Wrench& values) override
  {
    if (command_interfaces_.size() != WRENCH_SIZE)
    {
      return false;
    }
    values.force.x = command_interfaces_[0].get().get_value();
    values.force.y = command_interfaces_[1].get().get_value();
    values.force.z = command_interfaces_[2].get().get_value();
    values.torque.x = command_interfaces_[3].get().get_value();
    values.torque.y = command_interfaces_[4].get().get_value();
    values.torque.z = command_interfaces_[5].get().get_value();

    return true;
  }
};

}  // namespace acg_semantic_components
