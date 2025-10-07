/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_semantic_component_command_interface.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 27, 2025
 *
 * This class is a modified test file of the original
 * SemanticComponentCommandInterface class from ros2_control.
 *
 * -------------------------------------------------------------------
 */

// Gtest
#include <gtest/gtest.h>

// Standard libraries
#include <memory>  // for unique_ptr
#include <string>
#include <limits>

// ROS
#include <geometry_msgs/msg/pose.hpp>

// Class under test
#include "acg_semantic_components/semantic_component_command_interface.hpp"

/**
 * @brief Testable class that inherits from SemanticComponentCommandInterface to override the pure virtual functions allowing
 * the test cases to be run without implementing them.
 */
class TestableSemanticComponentCommandInterface : public semantic_components::SemanticComponentCommandInterface<geometry_msgs::msg::Pose>
{
public:
  TestableSemanticComponentCommandInterface(const std::string& name, const std::vector<std::string>& interface_names)
    : SemanticComponentCommandInterface(name, interface_names)
  {}

  ~TestableSemanticComponentCommandInterface() override = default;

private:
  // Note that set_values_from_message and get_values_as_message are a pure virtual function, hence the need to override them.
  // However, the test cases does not need to test them, so they are empty and their visibility is set to private.
  bool set_values_from_message(const geometry_msgs::msg::Pose& /* message */) override
  {
    return false;
  }
  bool get_values_as_message(geometry_msgs::msg::Pose& /* message */) override
  {
    return false;
  }
};
class SemanticComponentCommandInterfaceTest : public ::testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;

protected:
  std::string device_name_;
  std::vector<std::string> full_interface_names_;
  std::unique_ptr<TestableSemanticComponentCommandInterface> semantic_component_command_;
};

void SemanticComponentCommandInterfaceTest::SetUp()
{
  device_name_ = "test_semantic_component_command_interface";
}

void SemanticComponentCommandInterfaceTest::TearDown()
{
  semantic_component_command_.reset(nullptr);
}

/**
 * @brief Class that inherits from SemanticComponentCommandInterfaceTest to have a common setup for tests
 */
class ConfiguredSemanticComponentCommandInterfaceTest : public SemanticComponentCommandInterfaceTest
{
public:
  void SetUp() override;

protected:
  std::vector<double> exp_values_;
  std::size_t exp_size_{ 0 };
  std::vector<double> pose_values_;
  std::vector<std::string> expected_command_interface_names_;

  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> command_interfaces_;
  std::vector<hardware_interface::LoanedCommandInterface> loaned_command_interfaces_;
};

void ConfiguredSemanticComponentCommandInterfaceTest::SetUp()
{
  SemanticComponentCommandInterfaceTest::SetUp();
  exp_size_ = 7;

  // Define the interface names for the semantic component
  const std::vector<std::string> interface_names{ "position.x",    "position.y",    "position.z",   "orientation.x",
                                                  "orientation.y", "orientation.z", "orientation.w" };

  expected_command_interface_names_.reserve(exp_size_);
  for (const std::string& interface : interface_names)
  {
    expected_command_interface_names_.emplace_back(device_name_ + "/" + interface);
  }

  semantic_component_command_ = std::make_unique<TestableSemanticComponentCommandInterface>(device_name_, expected_command_interface_names_);
  exp_values_ = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7 };
  pose_values_ = std::vector<double>(exp_size_, std::numeric_limits<double>::quiet_NaN());

  command_interfaces_.reserve(exp_size_);
  loaned_command_interfaces_.reserve(exp_size_);

  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    command_interfaces_.emplace_back(std::make_shared<hardware_interface::CommandInterface>(device_name_, interface_names[i], &pose_values_[i]));
    loaned_command_interfaces_.emplace_back(*command_interfaces_[i]);
  }
}

/**
 * @brief Test the constructor of the SemanticComponentCommandInterface class with 0 interface names
 */
TEST_F(SemanticComponentCommandInterfaceTest, test_zero_interface_names_constructor)
{
  // Test zero interface names
  EXPECT_DEATH(TestableSemanticComponentCommandInterface(device_name_, std::vector<std::string>()), "");
}

/**
 * @brief Test the assign_loaned_command_interfaces function of the SemanticComponentCommandInterface class when the right size of command interfaces
 * is given
 */
TEST_F(ConfiguredSemanticComponentCommandInterfaceTest, test_assign_loaned_command_interfaces)
{
  // Assign interfaces
  EXPECT_TRUE(semantic_component_command_->assign_loaned_command_interfaces(loaned_command_interfaces_));

  // Release command interfaces
  semantic_component_command_->release_interfaces();
}

/**
 * @brief Test the assign_loaned_command_interfaces function of the SemanticComponentCommandInterface class when the command interfaces are set in a
 * jumbled order (but in the correct size)
 */
TEST_F(ConfiguredSemanticComponentCommandInterfaceTest, test_jumbled_assign_loaned_command_interfaces)
{
  // Create the command interfaces
  hardware_interface::CommandInterface position_x{ device_name_, "position.x", &pose_values_[0] };
  hardware_interface::CommandInterface position_y{ device_name_, "position.y", &pose_values_[1] };
  hardware_interface::CommandInterface position_z{ device_name_, "position.z", &pose_values_[2] };
  hardware_interface::CommandInterface orientation_x{ device_name_, "orientation.x", &pose_values_[3] };
  hardware_interface::CommandInterface orientation_y{ device_name_, "orientation.y", &pose_values_[4] };
  hardware_interface::CommandInterface orientation_z{ device_name_, "orientation.z", &pose_values_[5] };
  hardware_interface::CommandInterface orientation_w{ device_name_, "orientation.w", &pose_values_[6] };

  // Fill the vector of loaned command interfaces
  std::vector<hardware_interface::LoanedCommandInterface> loaned_command_interfaces;
  loaned_command_interfaces.reserve(7);
  loaned_command_interfaces.emplace_back(orientation_z);
  loaned_command_interfaces.emplace_back(position_x);
  loaned_command_interfaces.emplace_back(orientation_y);
  loaned_command_interfaces.emplace_back(position_z);
  loaned_command_interfaces.emplace_back(orientation_x);
  loaned_command_interfaces.emplace_back(orientation_w);
  loaned_command_interfaces.emplace_back(position_y);

  // Assign interfaces
  EXPECT_TRUE(semantic_component_command_->assign_loaned_command_interfaces(loaned_command_interfaces));

  // Release command interfaces
  semantic_component_command_->release_interfaces();
}

/**
 * @brief Test the assign_loaned_command_interfaces function of the SemanticComponentCommandInterface class when the wrong size of command interfaces
 * is given
 */
TEST_F(ConfiguredSemanticComponentCommandInterfaceTest, test_wrong_size_assign_loaned_command_interfaces)
{
  // Create the command interfaces
  hardware_interface::CommandInterface position_x{ device_name_, "position.x", &pose_values_[0] };
  hardware_interface::CommandInterface position_y{ device_name_, "position.y", &pose_values_[1] };

  // Fill the vector of loaned command interfaces
  std::vector<hardware_interface::LoanedCommandInterface> loaned_command_interfaces;
  loaned_command_interfaces.reserve(2);
  loaned_command_interfaces.emplace_back(position_x);
  loaned_command_interfaces.emplace_back(position_y);

  // Assign interfaces
  EXPECT_FALSE(semantic_component_command_->assign_loaned_command_interfaces(loaned_command_interfaces));
}

/**
 * @brief Test the get_command_interface_names function of the SemanticComponentCommandInterface class
 */
TEST_F(ConfiguredSemanticComponentCommandInterfaceTest, test_get_command_interface_names)
{
  ASSERT_TRUE(semantic_component_command_->assign_loaned_command_interfaces(loaned_command_interfaces_));

  // Get the interface names
  full_interface_names_ = semantic_component_command_->get_command_interface_names();

  // Validate the interface names
  ASSERT_EQ(full_interface_names_.size(), 7);
  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    EXPECT_EQ(full_interface_names_[i], expected_command_interface_names_[i]);
  }

  // Release command interfaces
  semantic_component_command_->release_interfaces();
}

/**
 * @brief Test the get_values function of the SemanticComponentCommandInterface class
 */
TEST_F(ConfiguredSemanticComponentCommandInterfaceTest, test_get_values)
{
  // Assign interfaces
  semantic_component_command_->assign_loaned_command_interfaces(loaned_command_interfaces_);

  // Set values for the command interfaces
  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    pose_values_[i] = exp_values_[i];
  }

  // Get values
  std::vector<double> pose_read(exp_size_, std::numeric_limits<double>::quiet_NaN());
  EXPECT_TRUE(semantic_component_command_->get_values(pose_read));

  // Validate the values
  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    EXPECT_EQ(pose_read[i], exp_values_[i]);
  }

  // Release command interfaces
  semantic_component_command_->release_interfaces();
}

/**
 * @brief Test the set_values and get_values functions of the SemanticComponentCommandInterface class
 */
TEST_F(ConfiguredSemanticComponentCommandInterfaceTest, test_set_values_and_get_values)
{
  // Assign interfaces
  semantic_component_command_->assign_loaned_command_interfaces(loaned_command_interfaces_);

  // Set values
  ASSERT_TRUE(semantic_component_command_->set_values(exp_values_));

  // Get values
  std::vector<double> pose_read(exp_size_, std::numeric_limits<double>::quiet_NaN());
  ASSERT_TRUE(semantic_component_command_->get_values(pose_read));

  // Validate the values
  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    EXPECT_EQ(pose_read[i], exp_values_[i]);
  }

  // Release command interfaces
  semantic_component_command_->release_interfaces();
}

/**
 * @brief Test the set_values function of the SemanticComponentCommandInterface class when the wrong size of values is given
 */
TEST_F(ConfiguredSemanticComponentCommandInterfaceTest, test_wrong_size_set_values)
{
  // Assign interfaces
  semantic_component_command_->assign_loaned_command_interfaces(loaned_command_interfaces_);

  // Set values
  std::vector<double> wrong_size_values(exp_size_ + 1, std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(semantic_component_command_->set_values(wrong_size_values));

  // Release command interfaces
  semantic_component_command_->release_interfaces();
}

/**
 * @brief Test the get_values function of the SemanticComponentCommandInterface class when the wrong size of values is given
 */
TEST_F(ConfiguredSemanticComponentCommandInterfaceTest, test_wrong_size_get_values)
{
  // Assign interfaces
  semantic_component_command_->assign_loaned_command_interfaces(loaned_command_interfaces_);

  // Set values
  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    pose_values_[i] = exp_values_[i];
  }

  // Get values
  std::vector<double> pose_read(exp_size_ + 1, std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(semantic_component_command_->get_values(pose_read));

  // Release command interfaces
  semantic_component_command_->release_interfaces();
}
