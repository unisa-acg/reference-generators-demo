/*
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   joint_position_command.hpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 17, 2025
 *
 * This class implements a joint position command semantic component.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "acg_semantic_components/semantic_component_command_interface.hpp"

namespace acg_semantic_components
{

class JointPositionCommand : public semantic_components::SemanticComponentCommandInterface<std::vector<double>>
{
private:
  std::vector<std::string> generate_interface_names_(const std::string& name, const std::size_t size, const std::vector<std::string>& interface_names)
  {
    if (!interface_names.empty())
    {
      if (interface_names.size() != size)
      {
        throw std::invalid_argument("JointPositionCommand: interface_names size (" + std::to_string(interface_names.size()) + ") must be equal to " +
                                    std::to_string(size));
      }
      return interface_names;
    }
    else
    {
      // Use standard interface names
      std::vector<std::string> interface_names;
      for (std::size_t i = 1; i <= size; ++i)
      {
        interface_names.emplace_back(name + "/joint" + std::to_string(i) + "/position");
      }
      return interface_names;
    }
  }

public:
  JointPositionCommand(const std::string& name, const std::size_t size = 0, const std::vector<std::string>& interface_names = {})
    : SemanticComponentCommandInterface(name, generate_interface_names_(name, size, interface_names))
  {}

  ~JointPositionCommand() = default;

  bool set_values_from_message(const std::vector<double>& values) override
  {
    if (values.size() != command_interfaces_.size())
    {
      return false;
    }

    for (std::size_t i = 0; i < values.size(); ++i)
    {
      command_interfaces_[i].get().set_value(values[i]);
    }
    return true;
  }

  bool get_values_as_message(std::vector<double>& values) override
  {
    if (values.size() != command_interfaces_.size())
    {
      return false;
    }

    for (std::size_t i = 0; i < values.size(); ++i)
    {
      values[i] = command_interfaces_[i].get().get_value();
    }
    return true;
  }
};

}  // namespace acg_semantic_components
