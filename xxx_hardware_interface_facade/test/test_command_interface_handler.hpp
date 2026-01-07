/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_command_interface_handler.hpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Mar 3, 2025
 *
 * This is a test file for the CommandInterfaceHandler class, where
 * a black box testing approach is used.
 * All functions of the CommandInterfaceHandler class are tested.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <vector>

#include <gtest/gtest.h>

#include <hardware_interface/loaned_state_interface.hpp>

#include <xxx_control_msgs/msg/joint_wrench_point.hpp>
#include <xxx_control_msgs/msg/task_space_point.hpp>

#include "xxx_hardware_interface_facade/command_interface_handler.hpp"

namespace xxx_hardware_interface_facade
{

struct CommandInterfaceTestParameters
{
public:
  std::vector<std::string> joint_names;
  std::string joint_controller_name;
  std::string task_space_controller_name;
  std::vector<std::string> joint_command_interfaces_names;
  std::vector<std::string> task_space_command_interfaces_names;
  std::map<std::string, bool> is_interface_expected;

  CommandInterfaceNamesOverrideConfig override_config;

  xxx_control_msgs::msg::JointWrenchPoint jwp;
  xxx_control_msgs::msg::TaskSpacePoint tsp;
};

class CommandInterfaceHandlerTest : public testing::TestWithParam<CommandInterfaceTestParameters>
{
public:
  void SetUp() override;

protected:
  // common variables used by the tests
  std::vector<std::string> joint_names_;
  std::string joint_controller_name_;
  std::string task_space_controller_name_;
  std::vector<std::string> joint_command_interfaces_names_;
  std::vector<std::string> task_space_command_interfaces_names_;
  std::map<std::string, bool> is_interface_expected_;
  CommandInterfaceNamesOverrideConfig override_config_;
  xxx_control_msgs::msg::JointWrenchPoint jwp_;
  xxx_control_msgs::msg::TaskSpacePoint tsp_;

  std::size_t num_joints_{ 0 };
  std::size_t total_interfaces_size_{ 0 };

  std::vector<std::string> expected_full_interface_names_;

  // The vector of doubles bound to the command interfaces
  std::vector<double> values_;
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> command_interfaces_;

  CommandInterfaceHandler command_interface_handler_;
};

/**
 * @brief This method returns the test parameters for testing a command interface handler with default names.
 *
 * @return CommandInterfaceTestParameters object.
 */
CommandInterfaceTestParameters getDefaultCommandInterfaceHandlerTestParams();

/**
 * @brief This method returns the test parameters for testing a command interface handler with overridden names.
 *
 * @return CommandInterfaceTestParameters object.
 */
CommandInterfaceTestParameters getOverrideCommandInterfaceHandlerTestParams();

}  // namespace xxx_hardware_interface_facade
