/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_wrench_command.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 27, 2025
 *
 * This is a test file for the WrenchCommand class.
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
#include <geometry_msgs/msg/wrench.hpp>

// Class under test
#include "acg_semantic_components/wrench_command.hpp"

class WrenchCommandTest : public ::testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;

protected:
  std::string device_name_;
  std::vector<std::string> interface_names_;
  std::unique_ptr<acg_semantic_components::WrenchCommand> wrench_command_;
};

void WrenchCommandTest::SetUp()
{
  device_name_ = "test_wrench_command";
}

void WrenchCommandTest::TearDown()
{
  wrench_command_.reset(nullptr);
}

/**
 * @brief Class that inherits from WrenchCommandTest to have a common setup for tests
 */
class ConfiguredWrenchCommandTest : public WrenchCommandTest
{
public:
  void SetUp() override;

protected:
  std::vector<double> exp_values_;
  std::size_t exp_size_{ 0 };
  std::vector<double> wrench_values_;
  std::vector<std::string> expected_cmd_interface_names_;

  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> command_interfaces_;
  std::vector<hardware_interface::LoanedCommandInterface> loaned_command_interfaces_;
};

void ConfiguredWrenchCommandTest::SetUp()
{
  WrenchCommandTest::SetUp();
  exp_size_ = 6;
  wrench_command_ = std::make_unique<acg_semantic_components::WrenchCommand>(device_name_);
  // Define the interface names for the twist command
  const std::vector<std::string> interface_names{ "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z" };

  expected_cmd_interface_names_.reserve(exp_size_);
  for (const std::string& interface : interface_names)
  {
    expected_cmd_interface_names_.emplace_back(device_name_ + "/" + interface);
  }

  exp_values_ = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 };
  wrench_values_ = std::vector<double>(exp_size_, std::numeric_limits<double>::quiet_NaN());

  command_interfaces_.reserve(exp_size_);
  loaned_command_interfaces_.reserve(exp_size_);

  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    command_interfaces_.emplace_back(std::make_shared<hardware_interface::CommandInterface>(device_name_, interface_names[i], &wrench_values_[i]));
    loaned_command_interfaces_.emplace_back(*command_interfaces_[i]);
  }
}

/**
 * @brief Test the constructor of the WrenchCommand class with wrong size of interface names
 */
TEST_F(WrenchCommandTest, test_wrong_size_interface_names)
{
  // Test wrong size of interface names
  EXPECT_THROW(std::make_unique<acg_semantic_components::WrenchCommand>(device_name_, std::vector<std::string>{ "wrong_size" }),
               std::invalid_argument);
}

/**
 * @brief Test the WrenchCommand class with custom interface names
 */
TEST_F(WrenchCommandTest, test_custom_interface_names)
{
  const std::vector<std::string> custom_interface_names{ "custom_force.x",  "custom_force.y",  "custom_force.z",
                                                         "custom_torque.x", "custom_torque.y", "custom_torque.z" };

  // Create device with custom interface names
  wrench_command_ = std::make_unique<acg_semantic_components::WrenchCommand>(device_name_, custom_interface_names);

  // Get the interface names
  interface_names_ = wrench_command_->get_command_interface_names();

  // Validate correct assignment of interface names
  EXPECT_EQ(interface_names_.size(), custom_interface_names.size());
  EXPECT_TRUE(std::equal(interface_names_.cbegin(), interface_names_.cend(), custom_interface_names.cbegin(), custom_interface_names.cend()));
}

/**
 * @brief Test the WrenchCommand class with default interface names and validate all the functionalities
 */
TEST_F(ConfiguredWrenchCommandTest, validate_all)
{
  // Get the interface names
  interface_names_ = wrench_command_->get_command_interface_names();

  // Validate the interface names
  ASSERT_EQ(interface_names_.size(), expected_cmd_interface_names_.size());
  EXPECT_TRUE(
      std::equal(interface_names_.cbegin(), interface_names_.cend(), expected_cmd_interface_names_.cbegin(), expected_cmd_interface_names_.cend()));

  // Assign the command interfaces
  ASSERT_TRUE(wrench_command_->assign_loaned_command_interfaces(loaned_command_interfaces_));

  // Create a message with the expected values
  geometry_msgs::msg::Wrench test_wrench_msg;
  test_wrench_msg.force.x = exp_values_[0];
  test_wrench_msg.force.y = exp_values_[1];
  test_wrench_msg.force.z = exp_values_[2];
  test_wrench_msg.torque.x = exp_values_[3];
  test_wrench_msg.torque.y = exp_values_[4];
  test_wrench_msg.torque.z = exp_values_[5];

  // Set values of the command interfaces
  ASSERT_TRUE(wrench_command_->set_values_from_message(test_wrench_msg));

  // Get values as message
  test_wrench_msg = geometry_msgs::msg::Wrench();
  ASSERT_TRUE(wrench_command_->get_values_as_message(test_wrench_msg));

  // Validate correct assignment of values of the command interfaces
  ASSERT_EQ(test_wrench_msg.force.x, exp_values_[0]);
  ASSERT_EQ(test_wrench_msg.force.y, exp_values_[1]);
  ASSERT_EQ(test_wrench_msg.force.z, exp_values_[2]);
  ASSERT_EQ(test_wrench_msg.torque.x, exp_values_[3]);
  ASSERT_EQ(test_wrench_msg.torque.y, exp_values_[4]);
  ASSERT_EQ(test_wrench_msg.torque.z, exp_values_[5]);

  // Release the command interfaces
  wrench_command_->release_interfaces();
}

/**
 * @brief Test the get_values_as_message function of the WrenchCommand class when a wrong size input is given
 */
TEST_F(ConfiguredWrenchCommandTest, test_wrong_size_get_values)
{
  // Create a few command interfaces
  hardware_interface::CommandInterface twist1{ device_name_, "force.x", &wrench_values_[0] };
  hardware_interface::CommandInterface twist2{ device_name_, "force.y", &wrench_values_[1] };

  // Fill the vector of loaned command interfaces
  std::vector<hardware_interface::LoanedCommandInterface> wrong_size_loan_command_interfaces;
  wrong_size_loan_command_interfaces.reserve(2);
  wrong_size_loan_command_interfaces.emplace_back(twist1);
  wrong_size_loan_command_interfaces.emplace_back(twist2);

  // Assign interfaces
  ASSERT_FALSE(wrench_command_->assign_loaned_command_interfaces(wrong_size_loan_command_interfaces));

  // Create a message
  geometry_msgs::msg::Wrench test_wrench_msg;

  // Get values as message
  EXPECT_FALSE(wrench_command_->get_values_as_message(test_wrench_msg));

  // Release the command interfaces
  wrench_command_->release_interfaces();
}

/**
 * @brief Test the set_values_from_message function of the WrenchCommand class when a wrong size input is given
 */
TEST_F(ConfiguredWrenchCommandTest, test_wrong_size_set_values)
{
  // Create a few command interfaces
  hardware_interface::CommandInterface twist1{ device_name_, "force.x", &wrench_values_[0] };
  hardware_interface::CommandInterface twist2{ device_name_, "force.y", &wrench_values_[1] };
  hardware_interface::CommandInterface twist3{ device_name_, "force.z", &wrench_values_[2] };

  // Fill the vector of loaned command interfaces
  std::vector<hardware_interface::LoanedCommandInterface> wrong_size_loan_command_interfaces;
  wrong_size_loan_command_interfaces.reserve(3);
  wrong_size_loan_command_interfaces.emplace_back(twist1);
  wrong_size_loan_command_interfaces.emplace_back(twist2);
  wrong_size_loan_command_interfaces.emplace_back(twist3);

  // Assign interfaces
  ASSERT_FALSE(wrench_command_->assign_loaned_command_interfaces(wrong_size_loan_command_interfaces));

  // Create a message with the wrong size
  geometry_msgs::msg::Wrench test_wrench_msg;
  test_wrench_msg.force.x = exp_values_[0];
  test_wrench_msg.force.y = exp_values_[1];
  test_wrench_msg.force.z = exp_values_[2];
  test_wrench_msg.torque.x = exp_values_[3];
  test_wrench_msg.torque.y = exp_values_[4];
  test_wrench_msg.torque.z = exp_values_[5];

  // Set values from the message
  EXPECT_FALSE(wrench_command_->set_values_from_message(test_wrench_msg));

  // Release the command interfaces
  wrench_command_->release_interfaces();
}
