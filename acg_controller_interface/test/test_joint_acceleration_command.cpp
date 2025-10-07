/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_joint_acceleration_command.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 27, 2025
 *
 * This is a test file for the JointAccelerationCommand class.
 *
 * -------------------------------------------------------------------
 */

// Gtest
#include <gtest/gtest.h>

// Standard libraries
#include <memory>  // for unique_ptr
#include <string>
#include <vector>
#include <limits>

// ROS
#include <hardware_interface/loaned_command_interface.hpp>

// Class under test
#include "acg_semantic_components/joint_acceleration_command.hpp"

class JointAccelerationCommandTest : public ::testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;

protected:
  std::string device_name_;
  std::vector<std::string> interface_names_;
  std::unique_ptr<acg_semantic_components::JointAccelerationCommand> joint_acceleration_command_;
};

void JointAccelerationCommandTest::SetUp()
{
  device_name_ = "test_joint_acceleration_command";
}

void JointAccelerationCommandTest::TearDown()
{
  joint_acceleration_command_.reset(nullptr);
}

/**
 * @brief Class that inherits from JointAccelerationCommandTest to have a common setup for tests
 */
class ConfiguredJointAccelerationCommandTest : public JointAccelerationCommandTest
{
public:
  void SetUp() override;

protected:
  std::vector<double> exp_values_;
  std::size_t exp_size_{ 0 };
  std::vector<double> accel_values_;
  std::vector<std::string> expected_cmd_interface_names_;

  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> command_interfaces_;
  std::vector<hardware_interface::LoanedCommandInterface> loaned_command_interfaces_;
};

void ConfiguredJointAccelerationCommandTest::SetUp()
{
  JointAccelerationCommandTest::SetUp();
  exp_size_ = 7;
  exp_values_ = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0 };
  joint_acceleration_command_ = std::make_unique<acg_semantic_components::JointAccelerationCommand>(device_name_, exp_size_);
  accel_values_ = std::vector<double>(exp_size_, std::numeric_limits<double>::quiet_NaN());

  expected_cmd_interface_names_.reserve(exp_size_);
  for (std::size_t i = 1; i <= exp_size_; ++i)
  {
    expected_cmd_interface_names_.emplace_back(device_name_ + "/joint" + std::to_string(i) + "/acceleration");
  }

  command_interfaces_.reserve(exp_size_);
  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    command_interfaces_.emplace_back(
        std::make_shared<hardware_interface::CommandInterface>(device_name_, "joint" + std::to_string(i + 1) + "/acceleration", &accel_values_[i]));
    loaned_command_interfaces_.emplace_back(*command_interfaces_[i]);
  }
}

/**
 * @brief Test the constructor of the JointAccelerationCommand class with wrong size of interface names
 */
TEST_F(JointAccelerationCommandTest, test_wrong_size_constructor)
{
  // Test the constructor with wrong size of interface names
  EXPECT_THROW(acg_semantic_components::JointAccelerationCommand(device_name_, 3, std::vector<std::string>{ "wrong_size" }), std::invalid_argument);
}

/**
 * @brief Test the JointAccelerationCommand class with custom interface names
 */
TEST_F(JointAccelerationCommandTest, test_custom_interface_names)
{
  const std::vector<std::string> custom_interface_names{ "custom_joint1", "custom_joint2", "custom_joint3", "custom_joint4",
                                                         "custom_joint5", "custom_joint6", "custom_joint7" };

  // Create device with custom interface names
  joint_acceleration_command_ =
      std::make_unique<acg_semantic_components::JointAccelerationCommand>(device_name_, custom_interface_names.size(), custom_interface_names);

  // Get the interface names
  interface_names_ = joint_acceleration_command_->get_command_interface_names();

  // Validate correct assignment of interface names
  EXPECT_EQ(interface_names_.size(), custom_interface_names.size());
  EXPECT_TRUE(std::equal(interface_names_.cbegin(), interface_names_.cend(), custom_interface_names.cbegin(), custom_interface_names.cend()));
}

/**
 * @brief Test the JointAccelerationCommand class with default interface names and validate all the functionalities
 */
TEST_F(ConfiguredJointAccelerationCommandTest, validate_all)
{
  // Get the interface names
  interface_names_ = joint_acceleration_command_->get_command_interface_names();

  // Validate the interface names
  ASSERT_EQ(interface_names_.size(), expected_cmd_interface_names_.size());
  ASSERT_TRUE(
      std::equal(interface_names_.cbegin(), interface_names_.cend(), expected_cmd_interface_names_.cbegin(), expected_cmd_interface_names_.cend()));

  // Assign interfaces
  ASSERT_TRUE(joint_acceleration_command_->assign_loaned_command_interfaces(loaned_command_interfaces_));

  // Set values of the command interfaces
  ASSERT_TRUE(joint_acceleration_command_->set_values_from_message(exp_values_));

  // Get values as message
  std::vector<double> joint_accel_values(exp_size_, std::numeric_limits<double>::quiet_NaN());
  ASSERT_TRUE(joint_acceleration_command_->get_values_as_message(joint_accel_values));

  // Validate correct assignment of values of the command interfaces
  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    EXPECT_EQ(joint_accel_values[i], exp_values_[i]);
  }

  // Release command interfaces
  joint_acceleration_command_->release_interfaces();
}

/**
 * @brief Test the get function of the JointAccelerationCommand class when a wrong size input is given to the get_values_as_message function
 */
TEST_F(ConfiguredJointAccelerationCommandTest, test_wrong_size_get_values)
{
  // Assign interfaces
  ASSERT_TRUE(joint_acceleration_command_->assign_loaned_command_interfaces(loaned_command_interfaces_));

  // Create a wrong size vector
  std::vector<double> wrong_size_values(loaned_command_interfaces_.size() + 1, std::numeric_limits<double>::quiet_NaN());

  // Get values as message
  EXPECT_FALSE(joint_acceleration_command_->get_values_as_message(wrong_size_values));

  // Release command interfaces
  joint_acceleration_command_->release_interfaces();
}

/**
 * @brief Test the set function of the JointAccelerationCommand class when a wrong size input is given to the set_values_from_message function
 */
TEST_F(ConfiguredJointAccelerationCommandTest, test_wrong_size_set_values)
{
  // Assign interfaces
  ASSERT_TRUE(joint_acceleration_command_->assign_loaned_command_interfaces(loaned_command_interfaces_));

  // Create a wrong size vector of values to set
  std::vector<double> wrong_size_values(loaned_command_interfaces_.size() + 1, std::numeric_limits<double>::quiet_NaN());

  // Set values from message
  EXPECT_FALSE(joint_acceleration_command_->set_values_from_message(wrong_size_values));

  // Release command interfaces
  joint_acceleration_command_->release_interfaces();
}
