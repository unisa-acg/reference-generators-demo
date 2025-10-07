/*
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   position_encoder.hpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 7, 2025
 *
 * This class implements a position encoder semantic component.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <semantic_components/semantic_component_interface.hpp>

namespace acg_semantic_components
{

class PositionEncoder : public semantic_components::SemanticComponentInterface<std::vector<double>>
{
public:
  PositionEncoder(const std::string& name, const std::size_t size = 0, const std::vector<std::string>& interface_names = {})
    : SemanticComponentInterface(name, size)
  {
    if (!interface_names.empty())
    {
      if (interface_names.size() != size)
      {
        throw std::invalid_argument("PositionEncoder: interface_names size (" + std::to_string(interface_names.size()) + ") must be equal to " +
                                    std::to_string(size));
      }
      interface_names_ = interface_names;
    }
    else
    {
      // Use standard interface names
      for (std::size_t i = 1; i <= size; ++i)
      {
        interface_names_.emplace_back(name_ + "/joint" + std::to_string(i) + "/position");
      }
    }
  }

  ~PositionEncoder() = default;

  std::vector<std::string> get_state_interface_names() const
  {
    return interface_names_;
  }

  bool get_values_as_message(std::vector<double>& values) const
  {
    if (values.size() != state_interfaces_.size())
    {
      return false;
    }

    for (std::size_t i = 0; i < values.size(); ++i)
    {
      values[i] = state_interfaces_[i].get().get_value();
    }
    return true;
  }
};

}  // namespace acg_semantic_components
