/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_position_encoder.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 28, 2025
 *
 * This is a test file for the PositionEncoder class.
 *
 * -------------------------------------------------------------------
 */

// Gtest
#include <gtest/gtest.h>

// Standard libraries
#include <memory>
#include <string>
#include <vector>
#include <limits>

// ROS
#include <hardware_interface/loaned_state_interface.hpp>

// Class under test
#include "acg_semantic_components/position_encoder.hpp"

class PositionEncoderTest : public ::testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;

protected:
  std::string sensor_name_;
  std::vector<std::string> interface_names_;
  std::unique_ptr<acg_semantic_components::PositionEncoder> position_encoder_;
};

void PositionEncoderTest::SetUp()
{
  sensor_name_ = "test_position_encoder";
}

void PositionEncoderTest::TearDown()
{
  position_encoder_.reset(nullptr);
}

/**
 * @brief Class that inherits from PositionEncoderTest to have a common setup for tests
 */
class ConfiguredPositionEncoderTest : public PositionEncoderTest
{
public:
  void SetUp() override;

protected:
  std::vector<double> exp_values_;
  std::vector<std::string> expected_state_interface_names_;
  std::size_t exp_size_{ 0 };

  std::vector<std::shared_ptr<hardware_interface::StateInterface>> state_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface> loaned_state_interfaces_;
};

void ConfiguredPositionEncoderTest::SetUp()
{
  PositionEncoderTest::SetUp();
  exp_size_ = 7;
  position_encoder_ = std::make_unique<acg_semantic_components::PositionEncoder>(sensor_name_, exp_size_);
  exp_values_ = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7 };

  expected_state_interface_names_.reserve(exp_size_);
  for (std::size_t i = 1; i <= exp_size_; ++i)
  {
    expected_state_interface_names_.emplace_back(sensor_name_ + "/joint" + std::to_string(i) + "/position");
  }

  state_interfaces_.reserve(exp_size_);
  loaned_state_interfaces_.reserve(exp_size_);
  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    state_interfaces_.emplace_back(
        std::make_shared<hardware_interface::StateInterface>(sensor_name_, "joint" + std::to_string(i + 1) + "/position", &exp_values_[i]));
    loaned_state_interfaces_.emplace_back(*state_interfaces_[i]);
  }
}

/**
 * @brief Test the constructor of the PositionEncoder class with wrong size of interface names
 */
TEST_F(PositionEncoderTest, test_wrong_size_constructor)
{
  // Test wrong size of interface names
  EXPECT_THROW(std::make_unique<acg_semantic_components::PositionEncoder>(sensor_name_, 3, std::vector<std::string>{ "wrong_size" }),
               std::invalid_argument);
}

/**
 * @brief Test the constructor of the PositionEncoder class with custom interface names
 */
TEST_F(PositionEncoderTest, test_custom_interface_names)
{
  const std::vector<std::string> custom_interface_names{ "custom_joint1/position", "custom_joint2/position", "custom_joint3/position",
                                                         "custom_joint4/position", "custom_joint5/position", "custom_joint6/position",
                                                         "custom_joint7/position" };

  // Create device with custom interface names
  position_encoder_ = std::make_unique<acg_semantic_components::PositionEncoder>(sensor_name_, custom_interface_names.size(), custom_interface_names);

  // Get the interface names
  interface_names_ = position_encoder_->get_state_interface_names();

  // Validate correct assignment of interface names
  ASSERT_EQ(interface_names_.size(), custom_interface_names.size());
  EXPECT_TRUE(std::equal(interface_names_.cbegin(), interface_names_.cend(), custom_interface_names.cbegin(), custom_interface_names.cend()));
}

/**
 * @brief Test the PositionEncoder class with default interface names and validate all the functionalities
 */
TEST_F(ConfiguredPositionEncoderTest, validate_all)
{
  // Get the interface names
  interface_names_ = position_encoder_->get_state_interface_names();

  // Validate the interface names
  ASSERT_EQ(interface_names_.size(), expected_state_interface_names_.size());
  ASSERT_TRUE(std::equal(interface_names_.cbegin(), interface_names_.cend(), expected_state_interface_names_.cbegin(),
                         expected_state_interface_names_.cend()));

  // Assign state interfaces
  ASSERT_TRUE(position_encoder_->assign_loaned_state_interfaces(loaned_state_interfaces_));

  // Create a vector to store the values of the state interfaces
  std::vector<double> joint_position_read(exp_size_, std::numeric_limits<double>::quiet_NaN());

  // Retrieve the values of the state interfaces
  ASSERT_TRUE(position_encoder_->get_values_as_message(joint_position_read));

  // Validate the values of the state interfaces
  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    EXPECT_EQ(joint_position_read[i], exp_values_[i]);
  }

  // Release state interfaces
  position_encoder_->release_interfaces();
}

/**
 * @brief Test the get function of the PositionEncoder class when a wrong size input is given to the get_values_as_message function
 */
TEST_F(ConfiguredPositionEncoderTest, test_wrong_size_get_values)
{
  // Assign interfaces
  ASSERT_TRUE(position_encoder_->assign_loaned_state_interfaces(loaned_state_interfaces_));

  // Create a vector to store the values of the state interfaces
  std::vector<double> joint_position_read(loaned_state_interfaces_.size() + 1, std::numeric_limits<double>::quiet_NaN());

  // Get values as message
  EXPECT_FALSE(position_encoder_->get_values_as_message(joint_position_read));

  // Release state interfaces
  position_encoder_->release_interfaces();
}
