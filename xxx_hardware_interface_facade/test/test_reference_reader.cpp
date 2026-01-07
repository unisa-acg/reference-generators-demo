/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_reference_reader.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Mar 5, 2025
 *
 * This is a test file for the ReferenceReader class, where
 * a black box testing approach is used.
 * All functions of the ReferenceReader class are tested.
 *
 * -------------------------------------------------------------------
 */

#include <gtest/gtest.h>

#include <xxx_control_msgs/msg/joint_wrench_point.hpp>
#include <xxx_control_msgs/msg/task_space_point.hpp>

#include "xxx_hardware_interface_facade/reference_reader.hpp"
#include "test_command_interface_handler.hpp"

namespace xxx_hardware_interface_facade
{

class ReferenceReaderTest : public CommandInterfaceHandlerTest
{
public:
  void SetUp() override;
  // method to export the command interfaces, as it is done in the controller
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces();

protected:
  // common variables used by the tests
  ReferenceReader reference_reader_;
};

/**
 * @brief This method is used to export the command interfaces, as it is done in the controller.
 */
std::vector<hardware_interface::CommandInterface> ReferenceReaderTest::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  std::size_t pos{ 0 };
  std::string prefix, name;
  for (std::size_t i = 0; i < expected_full_interface_names_.size(); ++i)
  {
    const std::string& interface {
      expected_full_interface_names_[i]
    };
    pos = interface.find_last_of("/");  // It is guaranteed that the interface name is in the form prefix/name
    prefix = interface.substr(0, pos);
    name = interface.substr(pos + 1);
    command_interfaces.emplace_back(prefix, name, &values_[i]);
  }
  return command_interfaces;
}

/**
 * @brief This method sets up the scenario given by the parameters of the test.
 */
void ReferenceReaderTest::SetUp()
{
  CommandInterfaceHandlerTest::SetUp();

  std::size_t total_interfaces_size_ = expected_full_interface_names_.size();
  values_.assign(total_interfaces_size_, 0.0);

  reference_reader_.configure_interfaces(joint_names_, joint_command_interfaces_names_, task_space_command_interfaces_names_, joint_controller_name_,
                                         task_space_controller_name_, override_config_);

  std::vector<std::string> available_command_interfaces{ reference_reader_.available_interfaces() };

  // assigning the command interfaces to the reference reader
  reference_reader_.assign_command_interfaces(on_export_reference_interfaces(), values_);

  // Setting the expected values in the values_ vector
  for (std::size_t i = 0; i < values_.size(); i++)
  {
    values_[i] = static_cast<double>(i);
  }
}

/**
 * @brief This test checks the correctness of the ReferenceReader class methods.
 */
TEST_P(ReferenceReaderTest, ReadAllInterfacesTaskSpace)
{
  // Creating a joint wrench point message to read from the command interfaces
  xxx_control_msgs::msg::JointWrenchPoint joint_command;
  joint_command.positions.assign(num_joints_, 0.0);
  joint_command.velocities.assign(num_joints_, 0.0);
  joint_command.accelerations.assign(num_joints_, 0.0);
  joint_command.effort.assign(num_joints_, 0.0);

  // Reading the joint wrench point message from the command interfaces
  reference_reader_.read_from_reference_interfaces(joint_command);

  // Checking if the joint command is correctly read from the command interfaces
  std::size_t index{ 0 };
  if (reference_reader_.has_joint_position_interface())
  {
    for (std::size_t i = 0; i < num_joints_; i++)
    {
      EXPECT_EQ(values_[index++], joint_command.positions[i]);
    }
  }
  if (reference_reader_.has_joint_velocity_interface())
  {
    for (std::size_t i = 0; i < num_joints_; i++)
    {
      EXPECT_EQ(values_[index++], joint_command.velocities[i]);
    }
  }
  if (reference_reader_.has_joint_acceleration_interface())
  {
    for (std::size_t i = 0; i < num_joints_; i++)
    {
      EXPECT_EQ(values_[index++], joint_command.accelerations[i]);
    }
  }
  if (reference_reader_.has_joint_effort_interface())
  {
    for (std::size_t i = 0; i < num_joints_; i++)
    {
      EXPECT_EQ(values_[index++], joint_command.effort[i]);
    }
  }
  if (reference_reader_.has_joint_wrench_interface())
  {
    EXPECT_EQ(values_[index++], joint_command.wrench.force.x);
    EXPECT_EQ(values_[index++], joint_command.wrench.force.y);
    EXPECT_EQ(values_[index++], joint_command.wrench.force.z);
    EXPECT_EQ(values_[index++], joint_command.wrench.torque.x);
    EXPECT_EQ(values_[index++], joint_command.wrench.torque.y);
    EXPECT_EQ(values_[index++], joint_command.wrench.torque.z);
  }

  // Creating a task space point message to read from the command interfaces
  xxx_control_msgs::msg::TaskSpacePoint task_space_command;

  // Reading the task space point message from the command interfaces
  reference_reader_.read_from_reference_interfaces(task_space_command);

  if (reference_reader_.has_task_space_pose_interface())
  {
    EXPECT_EQ(values_[index++], task_space_command.pose.position.x);
    EXPECT_EQ(values_[index++], task_space_command.pose.position.y);
    EXPECT_EQ(values_[index++], task_space_command.pose.position.z);
    EXPECT_EQ(values_[index++], task_space_command.pose.orientation.x);
    EXPECT_EQ(values_[index++], task_space_command.pose.orientation.y);
    EXPECT_EQ(values_[index++], task_space_command.pose.orientation.z);
    EXPECT_EQ(values_[index++], task_space_command.pose.orientation.w);
  }
  if (reference_reader_.has_task_space_twist_interface())
  {
    EXPECT_EQ(values_[index++], task_space_command.twist.linear.x);
    EXPECT_EQ(values_[index++], task_space_command.twist.linear.y);
    EXPECT_EQ(values_[index++], task_space_command.twist.linear.z);
    EXPECT_EQ(values_[index++], task_space_command.twist.angular.x);
    EXPECT_EQ(values_[index++], task_space_command.twist.angular.y);
    EXPECT_EQ(values_[index++], task_space_command.twist.angular.z);
  }
  if (reference_reader_.has_task_space_acceleration_interface())
  {
    EXPECT_EQ(values_[index++], task_space_command.acceleration.linear.x);
    EXPECT_EQ(values_[index++], task_space_command.acceleration.linear.y);
    EXPECT_EQ(values_[index++], task_space_command.acceleration.linear.z);
    EXPECT_EQ(values_[index++], task_space_command.acceleration.angular.x);
    EXPECT_EQ(values_[index++], task_space_command.acceleration.angular.y);
    EXPECT_EQ(values_[index++], task_space_command.acceleration.angular.z);
  }
  if (reference_reader_.has_task_space_wrench_interface())
  {
    EXPECT_EQ(values_[index++], task_space_command.wrench.force.x);
    EXPECT_EQ(values_[index++], task_space_command.wrench.force.y);
    EXPECT_EQ(values_[index++], task_space_command.wrench.force.z);
    EXPECT_EQ(values_[index++], task_space_command.wrench.torque.x);
    EXPECT_EQ(values_[index++], task_space_command.wrench.torque.y);
    EXPECT_EQ(values_[index++], task_space_command.wrench.torque.z);
  }
  if (reference_reader_.has_task_space_wrench_derivative_interface())
  {
    EXPECT_EQ(values_[index++], task_space_command.wrench_derivative.force.x);
    EXPECT_EQ(values_[index++], task_space_command.wrench_derivative.force.y);
    EXPECT_EQ(values_[index++], task_space_command.wrench_derivative.force.z);
    EXPECT_EQ(values_[index++], task_space_command.wrench_derivative.torque.x);
    EXPECT_EQ(values_[index++], task_space_command.wrench_derivative.torque.y);
    EXPECT_EQ(values_[index++], task_space_command.wrench_derivative.torque.z);
  }
}

// The same test parameters of the CommandInterfaceHandlerTestSuite can be used for the reference reader tests
INSTANTIATE_TEST_SUITE_P(ReferenceReaderTestSuite, ReferenceReaderTest,
                         testing::Values(getDefaultCommandInterfaceHandlerTestParams(), getOverrideCommandInterfaceHandlerTestParams()));

// *********** TESTS FOR UNCONFIGURED REFERENCE READER *********** //

class ReferenceReaderTestWhenNotConfigured : public testing::Test
{
protected:
  ReferenceReader reference_reader_;
  std::vector<double> values_;
};

/**
 * @brief Test the ReferenceReader class when the reference reader is not configured.
 */
TEST_F(ReferenceReaderTestWhenNotConfigured, TestReferenceReaderMethodsWhenNotConfigured)
{
  // trying to retrieve the available command interfaces from an unconfigured reference reader
  ASSERT_TRUE(reference_reader_.available_interfaces().empty());

  // trying to write the command interfaces from an unconfigured reference reader
  reference_reader_.assign_command_interfaces(std::vector<hardware_interface::CommandInterface>(), values_);

  xxx_control_msgs::msg::TaskSpacePoint task_space_command;
  reference_reader_.read_from_reference_interfaces(task_space_command);

  // Check that the read did not change the values of the task space command by comparing it with the default values
  EXPECT_EQ(task_space_command, xxx_control_msgs::msg::TaskSpacePoint());

  xxx_control_msgs::msg::JointWrenchPoint joint_command;
  reference_reader_.read_from_reference_interfaces(joint_command);

  // Check that the read did not change the values of the joint command by comparing it with the default values
  EXPECT_EQ(joint_command, xxx_control_msgs::msg::JointWrenchPoint());
}

}  // namespace xxx_hardware_interface_facade
