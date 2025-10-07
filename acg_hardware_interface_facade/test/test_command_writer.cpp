/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_command_writer.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Mar 5, 2025
 *
 * This is a test file for the CommandWriter class, where
 * a black box testing approach is used.
 * All functions of the CommandWriter class are tested.
 *
 * -------------------------------------------------------------------
 */

#include <gtest/gtest.h>

#include <acg_control_msgs/msg/joint_wrench_point.hpp>
#include <acg_control_msgs/msg/task_space_point.hpp>

#include "acg_hardware_interface_facade/command_writer.hpp"
#include "test_command_interface_handler.hpp"

namespace acg_hardware_interface_facade
{

class CommandWriterTest : public CommandInterfaceHandlerTest
{
public:
  void SetUp() override;

protected:
  // common variables used by the tests
  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces_loaned_;

  CommandWriter command_writer_;
};

/**
 * @brief This method sets up the scenario given by the parameters of the test.
 */
void CommandWriterTest::SetUp()
{
  CommandInterfaceHandlerTest::SetUp();

  total_interfaces_size_ = expected_full_interface_names_.size();
  values_.assign(total_interfaces_size_, 0.0);

  command_interfaces_.resize(total_interfaces_size_);
  for (std::size_t i = 0; i < total_interfaces_size_; ++i)
  {
    std::size_t pos = expected_full_interface_names_[i].find_last_of("/");
    std::string prefix = expected_full_interface_names_[i].substr(0, pos);
    std::string name = expected_full_interface_names_[i].substr(pos + 1);
    command_interfaces_[i] = std::make_shared<hardware_interface::CommandInterface>(prefix, name, &values_[i]);
    command_interfaces_loaned_.push_back(hardware_interface::LoanedCommandInterface(*command_interfaces_[i]));
  }

  command_writer_.configure_interfaces(joint_names_, joint_command_interfaces_names_, task_space_command_interfaces_names_, joint_controller_name_,
                                       task_space_controller_name_, override_config_);

  command_writer_.assign_loaned_command_interfaces(command_interfaces_loaned_);
}

/**
 * @brief Test all the CommandWriter class methods.
 */
TEST_P(CommandWriterTest, WriteOverrideConfigNamesTaskSpace)
{
  // Writing the joint wrench point message to the command interfaces
  command_writer_.write_to_command_interfaces(jwp_);

  // Writing the task space point message to the command interfaces
  command_writer_.write_to_command_interfaces(tsp_);

  // Checking if the values are correctly written to the command interfaces, in particular to the values_ vector
  std::size_t index{ 0 };
  if (command_writer_.has_joint_position_interface())
  {
    for (std::size_t i = 0; i < num_joints_; i++)
    {
      EXPECT_EQ(values_[index++], jwp_.positions[i]);
    }
  }
  if (command_writer_.has_joint_velocity_interface())
  {
    for (std::size_t i = 0; i < num_joints_; i++)
    {
      EXPECT_EQ(values_[index++], jwp_.velocities[i]);
    }
  }
  if (command_writer_.has_joint_acceleration_interface())
  {
    for (std::size_t i = 0; i < num_joints_; i++)
    {
      EXPECT_EQ(values_[index++], jwp_.accelerations[i]);
    }
  }
  if (command_writer_.has_joint_effort_interface())
  {
    for (std::size_t i = 0; i < num_joints_; i++)
    {
      EXPECT_EQ(values_[index++], jwp_.effort[i]);
    }
  }
  if (command_writer_.has_joint_wrench_interface())
  {
    EXPECT_EQ(values_[index++], jwp_.wrench.force.x);
    EXPECT_EQ(values_[index++], jwp_.wrench.force.y);
    EXPECT_EQ(values_[index++], jwp_.wrench.force.z);
    EXPECT_EQ(values_[index++], jwp_.wrench.torque.x);
    EXPECT_EQ(values_[index++], jwp_.wrench.torque.y);
    EXPECT_EQ(values_[index++], jwp_.wrench.torque.z);
  }
  if (command_writer_.has_task_space_pose_interface())
  {
    EXPECT_EQ(values_[index++], tsp_.pose.position.x);
    EXPECT_EQ(values_[index++], tsp_.pose.position.y);
    EXPECT_EQ(values_[index++], tsp_.pose.position.z);
    EXPECT_EQ(values_[index++], tsp_.pose.orientation.x);
    EXPECT_EQ(values_[index++], tsp_.pose.orientation.y);
    EXPECT_EQ(values_[index++], tsp_.pose.orientation.z);
    EXPECT_EQ(values_[index++], tsp_.pose.orientation.w);
  }
  if (command_writer_.has_task_space_twist_interface())
  {
    EXPECT_EQ(values_[index++], tsp_.twist.linear.x);
    EXPECT_EQ(values_[index++], tsp_.twist.linear.y);
    EXPECT_EQ(values_[index++], tsp_.twist.linear.z);
    EXPECT_EQ(values_[index++], tsp_.twist.angular.x);
    EXPECT_EQ(values_[index++], tsp_.twist.angular.y);
    EXPECT_EQ(values_[index++], tsp_.twist.angular.z);
  }
  if (command_writer_.has_task_space_acceleration_interface())
  {
    EXPECT_EQ(values_[index++], tsp_.acceleration.linear.x);
    EXPECT_EQ(values_[index++], tsp_.acceleration.linear.y);
    EXPECT_EQ(values_[index++], tsp_.acceleration.linear.z);
    EXPECT_EQ(values_[index++], tsp_.acceleration.angular.x);
    EXPECT_EQ(values_[index++], tsp_.acceleration.angular.y);
    EXPECT_EQ(values_[index++], tsp_.acceleration.angular.z);
  }
  if (command_writer_.has_task_space_wrench_interface())
  {
    EXPECT_EQ(values_[index++], tsp_.wrench.force.x);
    EXPECT_EQ(values_[index++], tsp_.wrench.force.y);
    EXPECT_EQ(values_[index++], tsp_.wrench.force.z);
    EXPECT_EQ(values_[index++], tsp_.wrench.torque.x);
    EXPECT_EQ(values_[index++], tsp_.wrench.torque.y);
    EXPECT_EQ(values_[index++], tsp_.wrench.torque.z);
  }
  if (command_writer_.has_task_space_wrench_derivative_interface())
  {
    EXPECT_EQ(values_[index++], tsp_.wrench_derivative.force.x);
    EXPECT_EQ(values_[index++], tsp_.wrench_derivative.force.y);
    EXPECT_EQ(values_[index++], tsp_.wrench_derivative.force.z);
    EXPECT_EQ(values_[index++], tsp_.wrench_derivative.torque.x);
    EXPECT_EQ(values_[index++], tsp_.wrench_derivative.torque.y);
    EXPECT_EQ(values_[index++], tsp_.wrench_derivative.torque.z);
  }
}

/**
 * @brief This method returns the test parameters for testing a command writer with default names.
 *
 * @return A CommandInterfaceTestParameters object.
 */
CommandInterfaceTestParameters getDefaultCommandWriterTestParams()
{
  CommandInterfaceTestParameters command_interface_parameters{ getDefaultCommandInterfaceHandlerTestParams() };
  std::size_t num_joints = command_interface_parameters.joint_names.size();

  // Creating a joint wrench point message to write to the command interfaces
  acg_control_msgs::msg::JointWrenchPoint jwp_;
  jwp_.positions.assign(num_joints, 1.0);
  jwp_.velocities.assign(num_joints, 2.0);
  jwp_.accelerations.assign(num_joints, 3.0);
  jwp_.effort.assign(num_joints, 4.0);
  jwp_.wrench.force.x = 5.0;
  jwp_.wrench.force.y = 6.0;
  jwp_.wrench.force.z = 7.0;
  jwp_.wrench.torque.x = 8.0;
  jwp_.wrench.torque.y = 9.0;
  jwp_.wrench.torque.z = 10.0;

  // Creating a task space point message to write to the command interfaces
  acg_control_msgs::msg::TaskSpacePoint tsp_;
  tsp_.pose.position.x = 1.0;
  tsp_.pose.position.y = 2.0;
  tsp_.pose.position.z = 3.0;
  tsp_.pose.orientation.x = 0.1;
  tsp_.pose.orientation.y = 0.2;
  tsp_.pose.orientation.z = 0.3;
  tsp_.pose.orientation.w = 0.4;
  tsp_.wrench.force.x = 0.11;
  tsp_.wrench.force.y = 0.22;
  tsp_.wrench.force.z = 0.33;
  tsp_.wrench.torque.x = 0.111;
  tsp_.wrench.torque.y = 0.222;
  tsp_.wrench.torque.z = 0.333;
  tsp_.wrench_derivative.force.x = 0.1111;
  tsp_.wrench_derivative.force.y = 0.2222;
  tsp_.wrench_derivative.force.z = 0.3333;
  tsp_.wrench_derivative.torque.x = 0.11111;
  tsp_.wrench_derivative.torque.y = 0.22222;
  tsp_.wrench_derivative.torque.z = 0.33333;

  command_interface_parameters.jwp = jwp_;
  command_interface_parameters.tsp = tsp_;

  return command_interface_parameters;
}

/**
 * @brief This method returns the test parameters for testing a command writer with overridden names.
 *
 * @return A CommandInterfaceTestParameters object.
 */
CommandInterfaceTestParameters getOverrideCommandWriterTestParams()
{
  CommandInterfaceTestParameters command_interface_parameters{ getOverrideCommandInterfaceHandlerTestParams() };

  // Creating a joint wrench point message to write to the command interfaces
  std::size_t num_joints = command_interface_parameters.joint_names.size();
  acg_control_msgs::msg::JointWrenchPoint jwp_;
  jwp_.positions.assign(num_joints, 1.0);
  jwp_.velocities.assign(num_joints, 2.0);

  // Creating a task space point message to write to the command interfaces
  acg_control_msgs::msg::TaskSpacePoint tsp_;
  tsp_.pose.position.x = 1.0;
  tsp_.pose.position.y = 2.0;
  tsp_.pose.position.z = 3.0;
  tsp_.pose.orientation.x = 0.1;
  tsp_.pose.orientation.y = 0.2;
  tsp_.pose.orientation.z = 0.3;
  tsp_.pose.orientation.w = 0.4;
  tsp_.wrench.force.x = 0.11;
  tsp_.wrench.force.y = 0.22;
  tsp_.wrench.force.z = 0.33;
  tsp_.wrench.torque.x = 0.111;
  tsp_.wrench.torque.y = 0.222;
  tsp_.wrench.torque.z = 0.333;
  tsp_.wrench_derivative.force.x = 0.1111;
  tsp_.wrench_derivative.force.y = 0.2222;
  tsp_.wrench_derivative.force.z = 0.3333;
  tsp_.wrench_derivative.torque.x = 0.11111;
  tsp_.wrench_derivative.torque.y = 0.22222;
  tsp_.wrench_derivative.torque.z = 0.33333;

  command_interface_parameters.jwp = jwp_;
  command_interface_parameters.tsp = tsp_;

  return command_interface_parameters;
}

INSTANTIATE_TEST_SUITE_P(CommandWriterTests, CommandWriterTest,
                         testing::Values(getDefaultCommandWriterTestParams(), getOverrideCommandWriterTestParams()));

}  // namespace acg_hardware_interface_facade
