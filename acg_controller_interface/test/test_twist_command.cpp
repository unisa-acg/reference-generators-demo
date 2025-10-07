/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_twist_command.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 27, 2025
 *
 * This is a test file for the TwistCommand class.
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
#include <geometry_msgs/msg/twist.hpp>

// Class under test
#include "acg_semantic_components/twist_command.hpp"

class TwistCommandTest : public ::testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;

protected:
  std::string device_name_;
  std::vector<std::string> interface_names_;
  std::unique_ptr<acg_semantic_components::TwistCommand> twist_command_;
};

void TwistCommandTest::SetUp()
{
  device_name_ = "test_twist_command";
}
void TwistCommandTest::TearDown()
{
  twist_command_.reset(nullptr);
}

/**
 * @brief Class that inherits from TwistCommandTest to have a common setup for tests
 */
class ConfiguredTwistCommandTest : public TwistCommandTest
{
public:
  void SetUp() override;

protected:
  std::vector<double> exp_values_;
  std::size_t exp_size_{ 0 };
  std::vector<double> twist_values_;
  std::vector<std::string> expected_cmd_interface_names_;

  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> command_interfaces_;
  std::vector<hardware_interface::LoanedCommandInterface> loaned_command_interfaces_;
};

void ConfiguredTwistCommandTest::SetUp()
{
  TwistCommandTest::SetUp();
  exp_size_ = 6;
  twist_command_ = std::make_unique<acg_semantic_components::TwistCommand>(device_name_);
  // Define the interface names for the twist command
  const std::vector<std::string> interface_names{ "linear_velocity.x",  "linear_velocity.y",  "linear_velocity.z",
                                                  "angular_velocity.x", "angular_velocity.y", "angular_velocity.z" };

  expected_cmd_interface_names_.reserve(exp_size_);
  for (const std::string& interface : interface_names)
  {
    expected_cmd_interface_names_.emplace_back(device_name_ + "/" + interface);
  }

  exp_values_ = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 };
  twist_values_ = std::vector<double>(exp_size_, std::numeric_limits<double>::quiet_NaN());

  command_interfaces_.reserve(exp_size_);
  loaned_command_interfaces_.reserve(exp_size_);

  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    command_interfaces_.emplace_back(std::make_shared<hardware_interface::CommandInterface>(device_name_, interface_names[i], &twist_values_[i]));
    loaned_command_interfaces_.emplace_back(*command_interfaces_[i]);
  }
}

/**
 * @brief Test the constructor of the TwistCommand class with wrong size of interface names
 */
TEST_F(TwistCommandTest, test_wrong_size_interface_names)
{
  // Test wrong size of interface names
  EXPECT_THROW(std::make_unique<acg_semantic_components::TwistCommand>(device_name_, std::vector<std::string>{ "wrong_size" }),
               std::invalid_argument);
}

/**
 * @brief Test the TwistCommand class with custom interface names
 */
TEST_F(TwistCommandTest, test_custom_interface_names)
{
  const std::vector<std::string> custom_interface_names{ "custom_linear_velocity.x",  "custom_linear_velocity.y",  "custom_linear_velocity.z",
                                                         "custom_angular_velocity.x", "custom_angular_velocity.y", "custom_angular_velocity.z" };

  // Create device with custom interface names
  twist_command_ = std::make_unique<acg_semantic_components::TwistCommand>(device_name_, custom_interface_names);

  // Get the interface names
  interface_names_ = twist_command_->get_command_interface_names();

  // Validate correct assignment of interface names
  EXPECT_EQ(interface_names_.size(), custom_interface_names.size());
  EXPECT_TRUE(std::equal(interface_names_.cbegin(), interface_names_.cend(), custom_interface_names.cbegin(), custom_interface_names.cend()));
}

/**
 * @brief Test the TwistCommand class with default interface names and validate all the functionalities
 */
TEST_F(ConfiguredTwistCommandTest, validate_all)
{
  // Get the interface names
  interface_names_ = twist_command_->get_command_interface_names();

  // Validate the interface names
  ASSERT_EQ(interface_names_.size(), expected_cmd_interface_names_.size());
  EXPECT_TRUE(
      std::equal(interface_names_.cbegin(), interface_names_.cend(), expected_cmd_interface_names_.cbegin(), expected_cmd_interface_names_.cend()));

  // Assign the command interfaces
  ASSERT_TRUE(twist_command_->assign_loaned_command_interfaces(loaned_command_interfaces_));

  // Create a message with the expected values
  geometry_msgs::msg::Twist test_twist_msg;
  test_twist_msg.linear.x = exp_values_[0];
  test_twist_msg.linear.y = exp_values_[1];
  test_twist_msg.linear.z = exp_values_[2];
  test_twist_msg.angular.x = exp_values_[3];
  test_twist_msg.angular.y = exp_values_[4];
  test_twist_msg.angular.z = exp_values_[5];

  // Set values of the command interfaces
  ASSERT_TRUE(twist_command_->set_values_from_message(test_twist_msg));

  // Get values as message
  test_twist_msg = geometry_msgs::msg::Twist();
  ASSERT_TRUE(twist_command_->get_values_as_message(test_twist_msg));

  // Validate correct assignment of values of the command interfaces
  ASSERT_EQ(test_twist_msg.linear.x, exp_values_[0]);
  ASSERT_EQ(test_twist_msg.linear.y, exp_values_[1]);
  ASSERT_EQ(test_twist_msg.linear.z, exp_values_[2]);
  ASSERT_EQ(test_twist_msg.angular.x, exp_values_[3]);
  ASSERT_EQ(test_twist_msg.angular.y, exp_values_[4]);
  ASSERT_EQ(test_twist_msg.angular.z, exp_values_[5]);

  // Release the command interfaces
  twist_command_->release_interfaces();
}

/**
 * @brief Test the get_values_as_message function of the TwistCommand class when a wrong size input is given
 */
TEST_F(ConfiguredTwistCommandTest, test_wrong_size_get_values)
{
  // Create a few command interfaces
  hardware_interface::CommandInterface twist1{ device_name_, "linear_velocity.x", &twist_values_[0] };
  hardware_interface::CommandInterface twist2{ device_name_, "linear_velocity.y", &twist_values_[1] };

  // Fill the vector of loaned command interfaces
  std::vector<hardware_interface::LoanedCommandInterface> wrong_size_loan_command_interfaces;
  wrong_size_loan_command_interfaces.reserve(2);
  wrong_size_loan_command_interfaces.emplace_back(twist1);
  wrong_size_loan_command_interfaces.emplace_back(twist2);

  // Assign interfaces
  ASSERT_FALSE(twist_command_->assign_loaned_command_interfaces(wrong_size_loan_command_interfaces));

  // Create a message
  geometry_msgs::msg::Twist test_twist_msg;

  // Get values as message
  EXPECT_FALSE(twist_command_->get_values_as_message(test_twist_msg));

  // Release the command interfaces
  twist_command_->release_interfaces();
}

/**
 * @brief Test the set_values_from_message function of the TwistCommand class when a wrong size input is given
 */
TEST_F(ConfiguredTwistCommandTest, test_wrong_size_set_values)
{
  // Create a few command interfaces
  hardware_interface::CommandInterface twist1{ device_name_, "linear_velocity.x", &twist_values_[0] };
  hardware_interface::CommandInterface twist2{ device_name_, "linear_velocity.y", &twist_values_[1] };
  hardware_interface::CommandInterface twist3{ device_name_, "linear_velocity.z", &twist_values_[2] };

  // Fill the vector of loaned command interfaces
  std::vector<hardware_interface::LoanedCommandInterface> wrong_size_loan_command_interfaces;
  wrong_size_loan_command_interfaces.reserve(3);
  wrong_size_loan_command_interfaces.emplace_back(twist1);
  wrong_size_loan_command_interfaces.emplace_back(twist2);
  wrong_size_loan_command_interfaces.emplace_back(twist3);

  // Assign interfaces
  ASSERT_FALSE(twist_command_->assign_loaned_command_interfaces(wrong_size_loan_command_interfaces));

  // Create a message with the wrong size
  geometry_msgs::msg::Twist test_twist_msg;
  test_twist_msg.linear.x = exp_values_[0];
  test_twist_msg.linear.y = exp_values_[1];
  test_twist_msg.linear.z = exp_values_[2];
  test_twist_msg.angular.x = exp_values_[3];
  test_twist_msg.angular.y = exp_values_[4];
  test_twist_msg.angular.z = exp_values_[5];

  // Set values from the message
  EXPECT_FALSE(twist_command_->set_values_from_message(test_twist_msg));

  // Release the command interfaces
  twist_command_->release_interfaces();
}
