/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_wrench_derivative_command.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 27, 2025
 *
 * This is a test file for the WrenchDerivativeCommand class.
 *
 * -------------------------------------------------------------------
 */

// Gtest
#include <gtest/gtest.h>

// Standard libraries
#include <memory>  // for unique_ptr
#include <string>
#include <vector>

// Class under test
#include "acg_semantic_components/wrench_derivative_command.hpp"

class WrenchDerivativeCommandTest : public ::testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;

protected:
  std::string device_name_;
  std::vector<std::string> interface_names_;
  std::unique_ptr<acg_semantic_components::WrenchDerivativeCommand> wrench_derivative_command_;
};

void WrenchDerivativeCommandTest::SetUp()
{
  device_name_ = "test_wrench_derivative_command";
}

void WrenchDerivativeCommandTest::TearDown()
{
  wrench_derivative_command_.reset(nullptr);
}

/**
 * @brief Test the constructor of the PositionEncoder class with wrong size of interface names
 */
TEST_F(WrenchDerivativeCommandTest, test_wrong_size_interface_names_constructor)
{
  EXPECT_THROW(acg_semantic_components::WrenchDerivativeCommand(device_name_, std::vector<std::string>{ "wrong_size" }), std::invalid_argument);
}

/**
 * @brief Test the WrenchDerivativeCommand class with custom interface names
 */
TEST_F(WrenchDerivativeCommandTest, test_custom_interface_names)
{
  const std::vector<std::string> custom_interface_names{ "custom_force_derivative.x",  "custom_force_derivative.y",  "custom_force_derivative.z",
                                                         "custom_torque_derivative.x", "custom_torque_derivative.y", "custom_torque_derivative.z" };

  // Create device with custom interface names
  wrench_derivative_command_ = std::make_unique<acg_semantic_components::WrenchDerivativeCommand>(device_name_, custom_interface_names);

  // Get the interface names
  interface_names_ = wrench_derivative_command_->get_command_interface_names();

  // Validate correct assignment of interface names
  EXPECT_EQ(interface_names_.size(), custom_interface_names.size());
  EXPECT_TRUE(std::equal(interface_names_.cbegin(), interface_names_.cend(), custom_interface_names.cbegin(), custom_interface_names.cend()));
}

/**
 * @brief Test the WrenchDerivativeCommand class with default interface names
 */
TEST_F(WrenchDerivativeCommandTest, test_default_interface_names)
{
  std::vector<std::string> default_interface_names;
  default_interface_names.reserve(6);
  default_interface_names.emplace_back(device_name_ + "/force_derivative.x");
  default_interface_names.emplace_back(device_name_ + "/force_derivative.y");
  default_interface_names.emplace_back(device_name_ + "/force_derivative.z");
  default_interface_names.emplace_back(device_name_ + "/torque_derivative.x");
  default_interface_names.emplace_back(device_name_ + "/torque_derivative.y");
  default_interface_names.emplace_back(device_name_ + "/torque_derivative.z");

  // Create device with default interface names
  wrench_derivative_command_ = std::make_unique<acg_semantic_components::WrenchDerivativeCommand>(device_name_);

  // Get the interface names
  interface_names_ = wrench_derivative_command_->get_command_interface_names();

  EXPECT_EQ(interface_names_.size(), default_interface_names.size());
  EXPECT_TRUE(std::equal(interface_names_.cbegin(), interface_names_.cend(), default_interface_names.cbegin(), default_interface_names.cend()));
}
