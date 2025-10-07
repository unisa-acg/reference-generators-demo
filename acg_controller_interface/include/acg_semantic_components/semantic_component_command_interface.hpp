// This file is a modified version of the original file semantic_component_command_interface.hpp from the package controller_interface (Jazzy
// distribution) of ros2_control.

// Copyright (c) 2024, Sherpa Mobile Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>
#include <vector>

#include <controller_interface/helpers.hpp>
#include <hardware_interface/loaned_command_interface.hpp>

namespace semantic_components
{
template <typename MessageInputType>
class SemanticComponentCommandInterface

/**
 * A base class for semantic components that manage command interfaces in a ROS2 controller.
 *
 * This class provides functionality to assign, release, and manipulate loaned command interfaces
 * for a semantic component. It also defines methods for setting and retrieving values from the
 * command interfaces, as well as abstract methods for interacting with message types.
 */
{
public:
  SemanticComponentCommandInterface(const std::string& name, const std::vector<std::string>& interface_names)
    : name_(name), interface_names_(interface_names)
  {
    assert(interface_names.size() > 0);
    command_interfaces_.reserve(interface_names.size());
  }

  virtual ~SemanticComponentCommandInterface() = default;

  /**
   * Assign loaned command interfaces on the controller start.
   *
   * \param[in] command_interfaces vector of command interfaces provided by the controller.
   */
  bool assign_loaned_command_interfaces(std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces)
  {
    return controller_interface::get_ordered_interfaces(command_interfaces, interface_names_, "", command_interfaces_);
  }

  /**
   * Release loaned command interfaces from the hardware. This function should be called when the controller is stopped.
   */
  void release_interfaces()
  {
    command_interfaces_.clear();
  }

  /**
   * The function should be used in "command_interface_configuration()" of a controller to provide
   * standardized command interface names semantic component.
   *
   * \default Default implementation defined command interfaces as "name/NR" where NR is number from 0 to size of values.
   * \return list of strings with command interface names for the semantic component.
   */
  const std::vector<std::string>& get_command_interface_names() const
  {
    return interface_names_;
  }

  /**
   * Set values to the command interfaces.
   *
   * The function of the original code has been modified because, in ROS2 Jazzy, the set_value method of LoanedCommandInterface returns a bool,
   * whereas in ROS2 Humble, it returns void.
   *
   * \return true if it set all the values, else false (i.e., invalid size).
   */
  bool set_values(const std::vector<double>& values)
  {
    // check we have sufficient memory
    if (values.size() != command_interfaces_.size())
    {
      return false;
    }
    // set values
    for (std::size_t i = 0; i < values.size(); ++i)
    {
      command_interfaces_[i].get().set_value(values[i]);
    }
    return true;
  }

  /**
   * Get values from the command interfaces.
   *
   * This function is not present in the original implementation.
   *
   * \return true if all values were retrieved successfully, false otherwise.
   */
  bool get_values(std::vector<double>& values) const
  {
    // check we have sufficient memory
    if (values.size() != command_interfaces_.size())
    {
      return false;
    }
    // insert all the values
    for (std::size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      values[i] = command_interfaces_[i].get().get_value();
    }
    return true;
  }

  /**
   * Set values from a message type.
   *
   * \return True if all values were set successfully, false otherwise.
   */
  virtual bool set_values_from_message(const MessageInputType& /* message */) = 0;

  /**
   * Get values from the command interfaces as a message type.
   *
   * This function is not present in the original implementation.
   *
   * \return True if all values were retrieved successfully, false otherwise.
   */
  virtual bool get_values_as_message(MessageInputType& /* message */) = 0;

protected:
  std::string name_;
  std::vector<std::string> interface_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> command_interfaces_;
};

}  // namespace semantic_components
