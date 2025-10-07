/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_accelerometer.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 28, 2025
 *
 * This is a test file for the Accelerometer class.
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
#include <hardware_interface/loaned_state_interface.hpp>

// Class under test
#include "acg_semantic_components/accelerometer.hpp"

class AccelerometerTest : public ::testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;

protected:
  std::string sensor_name_;
  std::vector<std::string> interface_names_;
  std::unique_ptr<acg_semantic_components::Accelerometer> accelerometer_;
};

void AccelerometerTest::SetUp()
{
  sensor_name_ = "test_accelerometer";
}

void AccelerometerTest::TearDown()
{
  accelerometer_.reset(nullptr);
}

/**
 * @brief Class that inherits from AccelerometerTest to have a common setup for tests
 */
class ConfiguredAccelerometerTest : public AccelerometerTest
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

void ConfiguredAccelerometerTest::SetUp()
{
  AccelerometerTest::SetUp();
  exp_size_ = 7;
  accelerometer_ = std::make_unique<acg_semantic_components::Accelerometer>(sensor_name_, exp_size_);
  exp_values_ = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0 };

  expected_state_interface_names_.reserve(exp_size_);
  for (std::size_t i = 1; i <= exp_size_; ++i)
  {
    expected_state_interface_names_.emplace_back(sensor_name_ + "/joint" + std::to_string(i) + "/acceleration");
  }

  state_interfaces_.reserve(exp_size_);
  loaned_state_interfaces_.reserve(exp_size_);
  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    state_interfaces_.emplace_back(
        std::make_shared<hardware_interface::StateInterface>(sensor_name_, "joint" + std::to_string(i + 1) + "/acceleration", &exp_values_[i]));
    loaned_state_interfaces_.emplace_back(*state_interfaces_[i]);
  }
}

/**
 * @brief Test the constructor of the Accelerometer class with wrong size of interface names
 */
TEST_F(AccelerometerTest, test_wrong_size_constructor)
{
  // Test wrong size of interface names
  EXPECT_THROW(std::make_unique<acg_semantic_components::Accelerometer>(sensor_name_, 3, std::vector<std::string>{ "wrong_size" }),
               std::invalid_argument);
}

/**
 * @brief Test the AccelerometerTest class with custom interface names
 */
TEST_F(AccelerometerTest, test_custom_interface_names)
{
  const std::vector<std::string> custom_interface_names{ "custom_joint1/acceleration", "custom_joint2/acceleration", "custom_joint3/acceleration",
                                                         "custom_joint4/acceleration", "custom_joint5/acceleration", "custom_joint6/acceleration",
                                                         "custom_joint7/acceleration" };

  // Create device with custom interface names
  accelerometer_ = std::make_unique<acg_semantic_components::Accelerometer>(sensor_name_, custom_interface_names.size(), custom_interface_names);

  // Get the interface names
  interface_names_ = accelerometer_->get_state_interface_names();

  // Validate correct assignment of interface names
  ASSERT_EQ(interface_names_.size(), custom_interface_names.size());
  EXPECT_TRUE(std::equal(interface_names_.cbegin(), interface_names_.cend(), custom_interface_names.cbegin(), custom_interface_names.cend()));
}

/**
 * @brief Test the Accelerometer class with default interface names and validate all the functionalities
 */
TEST_F(ConfiguredAccelerometerTest, validate_all)
{
  // Get the interface names
  interface_names_ = accelerometer_->get_state_interface_names();

  // Validate the interface names
  ASSERT_EQ(interface_names_.size(), expected_state_interface_names_.size());
  ASSERT_TRUE(std::equal(interface_names_.cbegin(), interface_names_.cend(), expected_state_interface_names_.cbegin(),
                         expected_state_interface_names_.cend()));

  // Assign state interfaces
  ASSERT_TRUE(accelerometer_->assign_loaned_state_interfaces(loaned_state_interfaces_));

  // Create a vector to store the values of the state interfaces
  std::vector<double> accelerometer_read(exp_size_, std::numeric_limits<double>::quiet_NaN());

  // Retrieve the values of the state interfaces
  ASSERT_TRUE(accelerometer_->get_values_as_message(accelerometer_read));

  // Validate the values of the state interfaces
  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    EXPECT_EQ(accelerometer_read[i], exp_values_[i]);
  }

  // Release the state interfaces
  accelerometer_->release_interfaces();
}

/**
 * @brief Test the get function of the Accelerometer class when the wrong size of the state interfaces is assigned
 */
TEST_F(ConfiguredAccelerometerTest, test_wrong_size_get_values)
{
  // Assign state interfaces
  ASSERT_TRUE(accelerometer_->assign_loaned_state_interfaces(loaned_state_interfaces_));

  // Create a vector to store the values of the state interfaces
  std::vector<double> accelerometer_read(loaned_state_interfaces_.size() + 1, std::numeric_limits<double>::quiet_NaN());

  // Retrieve the values of the state interfaces
  EXPECT_FALSE(accelerometer_->get_values_as_message(accelerometer_read));

  // release the state_interfaces_
  accelerometer_->release_interfaces();
}
