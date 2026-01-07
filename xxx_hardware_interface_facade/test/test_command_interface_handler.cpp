/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_command_interface_handler.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Mar 5, 2025
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <vector>

#include <gtest/gtest.h>

#include <hardware_interface/loaned_state_interface.hpp>

#include "xxx_hardware_interface_facade/command_interface_handler.hpp"
#include "test_command_interface_handler.hpp"

namespace xxx_hardware_interface_facade
{

/**
 * @brief This method sets up the scenario given by the parameters of the test.
 */
void CommandInterfaceHandlerTest::SetUp()
{
  const CommandInterfaceTestParameters params = GetParam();
  joint_names_ = params.joint_names;
  joint_controller_name_ = params.joint_controller_name;
  task_space_controller_name_ = params.task_space_controller_name;
  joint_command_interfaces_names_ = params.joint_command_interfaces_names;
  task_space_command_interfaces_names_ = params.task_space_command_interfaces_names;
  override_config_ = params.override_config;
  is_interface_expected_ = params.is_interface_expected;
  jwp_ = params.jwp;
  tsp_ = params.tsp;

  num_joints_ = joint_names_.size();

  // Creating a lambda function to avoid repeating the same code for each task space interfaces
  std::function<void(const std::string&, const std::string&)> add_task_space_interfaces =
      [&](const std::string& controller_name, const std::string& interface_name)
  {
    for (const std::string& suffix : std::vector<std::string>{ "x", "y", "z" })
    {
      expected_full_interface_names_.push_back(controller_name + "/" + interface_name + "." + suffix);
    }
  };

  // Creating a lambda function to avoid repeating the same code for each joint space interfaces
  std::function<void(const std::vector<std::string>&, const std::string&)> add_joint_space_interfaces =
      [&](const std::vector<std::string>& joint_names_, const std::string& interface)
  {
    for (const std::string& joint : joint_names_)
    {
      expected_full_interface_names_.push_back(joint_controller_name_ + "/" + joint + "/" + interface);
    }
  };

  for (const std::string& interface : joint_command_interfaces_names_)
  {
    if (interface == "position")
    {
      if (override_config_.joint_position_interface_names.empty())
      {
        add_joint_space_interfaces(joint_names_, "position");
      }
      else
      {
        expected_full_interface_names_.insert(expected_full_interface_names_.end(), override_config_.joint_position_interface_names.begin(),
                                              override_config_.joint_position_interface_names.end());
      }
      continue;
    }

    if (interface == "velocity")
    {
      if (override_config_.joint_velocity_interface_names.empty())
      {
        add_joint_space_interfaces(joint_names_, "velocity");
      }
      else
      {
        expected_full_interface_names_.insert(expected_full_interface_names_.end(), override_config_.joint_velocity_interface_names.begin(),
                                              override_config_.joint_velocity_interface_names.end());
      }
      continue;
    }

    if (interface == "acceleration")
    {
      if (override_config_.joint_acceleration_interface_names.empty())
      {
        add_joint_space_interfaces(joint_names_, "acceleration");
      }
      else
      {
        expected_full_interface_names_.insert(expected_full_interface_names_.end(), override_config_.joint_acceleration_interface_names.begin(),
                                              override_config_.joint_acceleration_interface_names.end());
      }
      continue;
    }

    if (interface == "effort")
    {
      if (override_config_.joint_effort_interface_names.empty())
      {
        add_joint_space_interfaces(joint_names_, "effort");
      }
      else
      {
        expected_full_interface_names_.insert(expected_full_interface_names_.end(), override_config_.joint_effort_interface_names.begin(),
                                              override_config_.joint_effort_interface_names.end());
      }
      continue;
    }

    if (interface == "wrench")
    {
      if (override_config_.joint_wrench_interface_names.empty())
      {
        // Note that force and torque have a task space interface, but they are contained in the JointWrenchPoint interface
        add_task_space_interfaces(joint_controller_name_, "force");
        add_task_space_interfaces(joint_controller_name_, "torque");
      }
      else
      {
        expected_full_interface_names_.insert(expected_full_interface_names_.end(), override_config_.joint_wrench_interface_names.begin(),
                                              override_config_.joint_wrench_interface_names.end());
      }
      continue;
    }
  }

  for (const std::string& interface : task_space_command_interfaces_names_)
  {
    if (interface == "pose")
    {
      // insert pose interface names
      if (override_config_.task_space_pose_interface_names.empty())
      {
        add_task_space_interfaces(task_space_controller_name_, "position");
        add_task_space_interfaces(task_space_controller_name_, "orientation");
        expected_full_interface_names_.push_back(task_space_controller_name_ + "/orientation.w");
      }
      else
      {
        expected_full_interface_names_.insert(expected_full_interface_names_.end(), override_config_.task_space_pose_interface_names.begin(),
                                              override_config_.task_space_pose_interface_names.end());
      }
      continue;
    }

    if (interface == "twist")
    {
      // insert twist interface names
      if (override_config_.task_space_twist_interface_names.empty())
      {
        add_task_space_interfaces(task_space_controller_name_, "linear_velocity");
        add_task_space_interfaces(task_space_controller_name_, "angular_velocity");
      }
      else
      {
        expected_full_interface_names_.insert(expected_full_interface_names_.end(), override_config_.task_space_twist_interface_names.begin(),
                                              override_config_.task_space_twist_interface_names.end());
      }
      continue;
    }

    if (interface == "acceleration")
    {
      // insert acceleration interface names
      if (override_config_.task_space_acceleration_interface_names.empty())
      {
        add_task_space_interfaces(task_space_controller_name_, "linear_acceleration");
        add_task_space_interfaces(task_space_controller_name_, "angular_acceleration");
      }
      else
      {
        expected_full_interface_names_.insert(expected_full_interface_names_.end(), override_config_.task_space_acceleration_interface_names.begin(),
                                              override_config_.task_space_acceleration_interface_names.end());
      }
      continue;
    }

    if (interface == "wrench")
    {
      // insert wrench interface names
      if (override_config_.task_space_wrench_interface_names.empty())
      {
        add_task_space_interfaces(task_space_controller_name_, "force");
        add_task_space_interfaces(task_space_controller_name_, "torque");
      }
      else
      {
        expected_full_interface_names_.insert(expected_full_interface_names_.end(), override_config_.task_space_wrench_interface_names.begin(),
                                              override_config_.task_space_wrench_interface_names.end());
      }
      continue;
    }

    if (interface == "wrench_derivative")
    {
      // insert wrench_derivative interface names.
      if (override_config_.task_space_wrench_derivative_interface_names.empty())
      {
        add_task_space_interfaces(task_space_controller_name_, "force_derivative");
        add_task_space_interfaces(task_space_controller_name_, "torque_derivative");
      }
      else
      {
        expected_full_interface_names_.insert(expected_full_interface_names_.end(),
                                              override_config_.task_space_wrench_derivative_interface_names.begin(),
                                              override_config_.task_space_wrench_derivative_interface_names.end());
      }
      continue;
    }
  }
}

/**
 * @brief This test checks the correctness of the names of the command interfaces.
 */
TEST_P(CommandInterfaceHandlerTest, TestCommandInterfacesNames)
{
  command_interface_handler_.configure_interfaces(joint_names_, joint_command_interfaces_names_, task_space_command_interfaces_names_,
                                                  joint_controller_name_, task_space_controller_name_, override_config_);

  std::vector<std::string> available_command_interfaces = command_interface_handler_.available_interfaces();

  ASSERT_EQ(available_command_interfaces.size(), expected_full_interface_names_.size());

  for (std::size_t i = 0; i < available_command_interfaces.size(); i++)
  {
    EXPECT_EQ(available_command_interfaces[i], expected_full_interface_names_[i]);
  }
}

/**
 * @brief This test checks the availability of the command interfaces.
 */
TEST_P(CommandInterfaceHandlerTest, TestInterfacesAvailability)
{
  command_interface_handler_.configure_interfaces(joint_names_, joint_command_interfaces_names_, task_space_command_interfaces_names_,
                                                  joint_controller_name_, task_space_controller_name_, override_config_);

  // Check if the command interface handler set the command interfaces correctly
  EXPECT_EQ(command_interface_handler_.has_joint_position_interface(), is_interface_expected_["position"]);
  EXPECT_EQ(command_interface_handler_.has_joint_velocity_interface(), is_interface_expected_["velocity"]);
  EXPECT_EQ(command_interface_handler_.has_joint_acceleration_interface(), is_interface_expected_["acceleration"]);
  EXPECT_EQ(command_interface_handler_.has_joint_effort_interface(), is_interface_expected_["effort"]);
  EXPECT_EQ(command_interface_handler_.has_joint_wrench_interface(), is_interface_expected_["joint_wrench"]);
  EXPECT_EQ(command_interface_handler_.has_task_space_pose_interface(), is_interface_expected_["pose"]);
  EXPECT_EQ(command_interface_handler_.has_task_space_twist_interface(), is_interface_expected_["twist"]);
  EXPECT_EQ(command_interface_handler_.has_task_space_acceleration_interface(), is_interface_expected_["acceleration"]);
  EXPECT_EQ(command_interface_handler_.has_task_space_wrench_interface(), is_interface_expected_["task_wrench"]);
  EXPECT_EQ(command_interface_handler_.has_task_space_wrench_derivative_interface(), is_interface_expected_["wrench_derivative"]);
}

/**
 * @brief Refer to the header file for a description of this method.
 */
CommandInterfaceTestParameters getDefaultCommandInterfaceHandlerTestParams()
{
  CommandInterfaceTestParameters params;

  params.joint_names = { "joint1", "joint2", "joint3" };
  params.joint_controller_name = "joint_space_controller";
  params.task_space_controller_name = "task_space_controller";
  params.joint_command_interfaces_names = { "position", "velocity", "acceleration", "effort", "wrench" };
  params.task_space_command_interfaces_names = { "pose", "twist", "acceleration", "wrench", "wrench_derivative" };

  params.is_interface_expected = {
    { "position", true }, { "velocity", true }, { "acceleration", true }, { "effort", true },      { "joint_wrench", true },
    { "pose", true },     { "twist", true },    { "acceleration", true }, { "task_wrench", true }, { "wrench_derivative", true }
  };
  return params;
}

/**
 * @brief Refer to the header file for a description of this method.
 */
CommandInterfaceTestParameters getOverrideCommandInterfaceHandlerTestParams()
{
  CommandInterfaceTestParameters params;
  params.joint_names = { "robot_joint1", "robot_joint2", "robot_joint3" };
  params.joint_controller_name = "no_joint_controller";
  params.task_space_controller_name = "no_task_space_controller";
  params.joint_command_interfaces_names = { "position", "velocity" };
  params.task_space_command_interfaces_names = { "pose", "wrench", "wrench_derivative" };
  params.override_config.joint_position_interface_names = { "robot_joint1/position", "robot_joint2/position", "robot_joint3/position" };
  params.override_config.joint_velocity_interface_names = { "robot_joint1/velocity", "robot_joint2/velocity", "robot_joint3/velocity" };
  params.override_config.task_space_pose_interface_names = { "pose/x", "pose/y", "pose/z", "pose/rx", "pose/ry", "pose/rz", "pose/rw" };
  params.override_config.task_space_wrench_interface_names = { "wrench/force/x",  "wrench/force/y",  "wrench/force/z",
                                                               "wrench/torque/x", "wrench/torque/y", "wrench/torque/z" };
  params.override_config.task_space_wrench_derivative_interface_names = { "wrench_derivative/force/x",  "wrench_derivative/force/y",
                                                                          "wrench_derivative/force/z",  "wrench_derivative/torque/x",
                                                                          "wrench_derivative/torque/y", "wrench_derivative/torque/z" };

  params.is_interface_expected = { { "position", true },         { "velocity", true },      { "acceleration", false },
                                   { "effort", false },          { "joint_wrench", false }, { "pose", true },
                                   { "twist", false },           { "acceleration", false }, { "task_wrench", true },
                                   { "wrench_derivative", true } };
  return params;
}

INSTANTIATE_TEST_SUITE_P(CommandInterfaceHandlerTestSuite, CommandInterfaceHandlerTest,
                         testing::Values(getDefaultCommandInterfaceHandlerTestParams(), getOverrideCommandInterfaceHandlerTestParams()));

// *********** TESTS FOR UNCONFIGURED COMMAND INTERFACE HANDLER *********** //
class EmptyCommandInterfaceHandlerTest : public testing::Test
{
protected:
  CommandInterfaceHandler command_interface_handler_;
};

/**
 * @brief This test tries to use the CommandInterfaceHandler class methods without configuring the command interfaces.
 */
TEST_F(EmptyCommandInterfaceHandlerTest, TestCommandInterfaceHandlerMethodsWhenNotConfigured)
{
  // trying to retrieve the available command interfaces from an unconfigured command interface handler
  ASSERT_TRUE(command_interface_handler_.available_interfaces().empty());

  EXPECT_FALSE(command_interface_handler_.has_joint_position_interface());
  EXPECT_FALSE(command_interface_handler_.has_joint_velocity_interface());
  EXPECT_FALSE(command_interface_handler_.has_joint_acceleration_interface());
  EXPECT_FALSE(command_interface_handler_.has_joint_effort_interface());
  EXPECT_FALSE(command_interface_handler_.has_joint_wrench_interface());
  EXPECT_FALSE(command_interface_handler_.has_task_space_pose_interface());
  EXPECT_FALSE(command_interface_handler_.has_task_space_twist_interface());
  EXPECT_FALSE(command_interface_handler_.has_task_space_acceleration_interface());
  EXPECT_FALSE(command_interface_handler_.has_task_space_wrench_interface());
  EXPECT_FALSE(command_interface_handler_.has_task_space_wrench_derivative_interface());
}

}  // namespace xxx_hardware_interface_facade
