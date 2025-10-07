/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_accel_command.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 27, 2025
 *
 * This is a test file for the AccelCommand class.
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
#include <geometry_msgs/msg/accel.hpp>

// Class under test
#include "acg_semantic_components/accel_command.hpp"

class AccelCommandTest : public ::testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;

protected:
  std::string device_name_;
  std::vector<std::string> interface_names_;
  std::unique_ptr<acg_semantic_components::AccelCommand> accel_command_;
};

void AccelCommandTest::SetUp()
{
  device_name_ = "test_accel_command";
}

void AccelCommandTest::TearDown()
{
  accel_command_.reset(nullptr);
}

/**
 * @brief Class that inherits from AccelCommandTest to have a common setup for tests
 */
class ConfiguredAccelCommandTest : public AccelCommandTest
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

void ConfiguredAccelCommandTest::SetUp()
{
  AccelCommandTest::SetUp();
  exp_size_ = 6;
  accel_command_ = std::make_unique<acg_semantic_components::AccelCommand>(device_name_);
  exp_values_ = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 };
  accel_values_ = std::vector<double>(exp_size_, std::numeric_limits<double>::quiet_NaN());

  const std::vector<std::string> interface_names{ "linear_acceleration.x",  "linear_acceleration.y",  "linear_acceleration.z",
                                                  "angular_acceleration.x", "angular_acceleration.y", "angular_acceleration.z" };

  expected_cmd_interface_names_.reserve(exp_size_);
  for (const std::string& interface : interface_names)
  {
    expected_cmd_interface_names_.emplace_back(device_name_ + "/" + interface);
  }

  command_interfaces_.reserve(exp_size_);
  loaned_command_interfaces_.reserve(exp_size_);

  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    command_interfaces_.emplace_back(std::make_shared<hardware_interface::CommandInterface>(device_name_, interface_names[i], &accel_values_[i]));
    loaned_command_interfaces_.emplace_back(*command_interfaces_[i]);
  }
}

/**
 * @brief Test the constructor of the AccelCommand class when the number of interface names are wrong
 */
TEST_F(AccelCommandTest, test_wrong_size_constructor)
{
  // Test wrong size of interface names
  EXPECT_THROW(acg_semantic_components::AccelCommand(device_name_, std::vector<std::string>{ "wrong_size" }), std::invalid_argument);
}

/**
 * @brief Test the AccelCommand class with custom interface names
 */
TEST_F(AccelCommandTest, test_custom_interface_names)
{
  const std::vector<std::string> custom_interface_names{ "custom_linear_acceleration.x",  "custom_linear_acceleration.y",
                                                         "custom_linear_acceleration.z",  "custom_angular_acceleration.x",
                                                         "custom_angular_acceleration.y", "custom_angular_acceleration.z" };

  // Create device with custom interface names
  accel_command_ = std::make_unique<acg_semantic_components::AccelCommand>(device_name_, custom_interface_names);

  // Get the interface names
  interface_names_ = accel_command_->get_command_interface_names();

  // Validate correct assignment of interface names
  EXPECT_EQ(interface_names_.size(), custom_interface_names.size());
  EXPECT_TRUE(std::equal(interface_names_.cbegin(), interface_names_.cend(), custom_interface_names.cbegin(), custom_interface_names.cend()));
}

/**
 * @brief Test the AccelCommand class with default interface names and validate all the functionalities
 */
TEST_F(ConfiguredAccelCommandTest, validate_all)
{
  // Get the interface names
  interface_names_ = accel_command_->get_command_interface_names();

  // Validate the interface names
  ASSERT_EQ(interface_names_.size(), expected_cmd_interface_names_.size());
  ASSERT_TRUE(
      std::equal(interface_names_.cbegin(), interface_names_.cend(), expected_cmd_interface_names_.cbegin(), expected_cmd_interface_names_.cend()));

  // Assign interfaces
  ASSERT_TRUE(accel_command_->assign_loaned_command_interfaces(loaned_command_interfaces_));

  // Create a message with the expected values
  geometry_msgs::msg::Accel test_accel_msg;
  test_accel_msg.linear.x = exp_values_[0];
  test_accel_msg.linear.y = exp_values_[1];
  test_accel_msg.linear.z = exp_values_[2];
  test_accel_msg.angular.x = exp_values_[3];
  test_accel_msg.angular.y = exp_values_[4];
  test_accel_msg.angular.z = exp_values_[5];

  // Set values of the command interfaces
  ASSERT_TRUE(accel_command_->set_values_from_message(test_accel_msg));

  // Get values as message
  test_accel_msg = geometry_msgs::msg::Accel();
  ASSERT_TRUE(accel_command_->get_values_as_message(test_accel_msg));

  // Validate correct assignment of values of the command interfaces
  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    EXPECT_EQ(accel_values_[i], exp_values_[i]);
  }

  // Release command interfaces
  accel_command_->release_interfaces();
}

/**
 * @brief Test the get function of the AccelCommand class when the wrong input is given to the function get_values_as_message
 */
TEST_F(ConfiguredAccelCommandTest, test_wrong_size_get_values)
{
  // Create only one command interface
  hardware_interface::CommandInterface linear_acceleration_x{ device_name_, "linear_acceleration.x", &accel_values_[0] };

  // Create a wrong size of loaned command interfaces
  std::vector<hardware_interface::LoanedCommandInterface> wrong_size_loan_command_interfaces;
  wrong_size_loan_command_interfaces.emplace_back(linear_acceleration_x);

  EXPECT_FALSE(accel_command_->assign_loaned_command_interfaces(wrong_size_loan_command_interfaces));
  geometry_msgs::msg::Accel test_accel_msg;
  EXPECT_FALSE(accel_command_->get_values_as_message(test_accel_msg));

  // Release command interfaces
  accel_command_->release_interfaces();
}

/**
 * @brief Test the set function of the AccelCommand class when a wrong size input is given to the set_values_from_message function
 */
TEST_F(ConfiguredAccelCommandTest, test_wrong_size_set_values)
{
  // Create a few command interfaces
  hardware_interface::CommandInterface linear_acceleration_x{ device_name_, "linear_acceleration.x", &accel_values_[0] };
  hardware_interface::CommandInterface linear_acceleration_y{ device_name_, "linear_acceleration.y", &accel_values_[1] };

  // Create a wrong size of loaned command interfaces
  std::vector<hardware_interface::LoanedCommandInterface> wrong_size_loan_command_interfaces;
  wrong_size_loan_command_interfaces.emplace_back(linear_acceleration_x);
  wrong_size_loan_command_interfaces.emplace_back(linear_acceleration_y);

  EXPECT_FALSE(accel_command_->assign_loaned_command_interfaces(wrong_size_loan_command_interfaces));
  geometry_msgs::msg::Accel test_accel_msg;
  test_accel_msg.linear.x = exp_values_[0];
  test_accel_msg.linear.y = exp_values_[1];
  test_accel_msg.linear.z = exp_values_[2];
  test_accel_msg.angular.x = exp_values_[3];
  test_accel_msg.angular.y = exp_values_[4];
  test_accel_msg.angular.z = exp_values_[5];
  EXPECT_FALSE(accel_command_->set_values_from_message(test_accel_msg));

  // Release command interfaces
  accel_command_->release_interfaces();
}
