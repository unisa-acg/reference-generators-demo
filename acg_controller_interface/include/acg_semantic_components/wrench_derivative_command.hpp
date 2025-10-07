/*
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   wrench_derivative_command.hpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 17, 2025
 *
 * This class implements a wrench derivative command semantic
 * component.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include "acg_semantic_components/wrench_command.hpp"

namespace acg_semantic_components
{

class WrenchDerivativeCommand : public WrenchCommand
{
private:
  std::vector<std::string> generate_interface_names_(const std::string& name, const std::vector<std::string>& interface_names)
  {
    if (!interface_names.empty())
    {
      if (interface_names.size() != WRENCH_SIZE)
      {
        throw std::invalid_argument("WrenchDerivativeCommand: interface_names size (" + std::to_string(interface_names.size()) +
                                    ") must be equal to " + std::to_string(WRENCH_SIZE));
      }
      return interface_names;
    }
    else
    {
      // Return standard interface names
      return { { name + "/force_derivative.x" },  { name + "/force_derivative.y" },  { name + "/force_derivative.z" },
               { name + "/torque_derivative.x" }, { name + "/torque_derivative.y" }, { name + "/torque_derivative.z" } };
    }
  }

public:
  WrenchDerivativeCommand(const std::string& name, const std::vector<std::string>& interface_names = {})
    : WrenchCommand(name, generate_interface_names_(name, interface_names))
  {}
};

}  // namespace acg_semantic_components
