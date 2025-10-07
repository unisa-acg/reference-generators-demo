/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_pose_command.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 27, 2025
 *
 * This is a test file for the PoseCommand class.
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
#include <geometry_msgs/msg/pose.hpp>

// Class under test
#include "acg_semantic_components/pose_command.hpp"

class PoseCommandTest : public ::testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;

protected:
  std::string device_name_;
  std::vector<std::string> interface_names_;
  std::unique_ptr<acg_semantic_components::PoseCommand> pose_command_;
};

void PoseCommandTest::SetUp()
{
  device_name_ = "test_pose_command";
}

void PoseCommandTest::TearDown()
{
  pose_command_.reset(nullptr);
}

/**
 * @brief Class that inherits from PoseCommandTest to have a common setup for tests
 */
class ConfiguredPoseCommandTest : public PoseCommandTest
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

void ConfiguredPoseCommandTest::SetUp()
{
  PoseCommandTest::SetUp();
  exp_size_ = 7;
  pose_command_ = std::make_unique<acg_semantic_components::PoseCommand>(device_name_);
  exp_values_ = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7 };
  pose_values_ = std::vector<double>(exp_size_, std::numeric_limits<double>::quiet_NaN());

  // Define the interface names for the pose semantic component command
  const std::vector<std::string> interface_names{ "position.x",    "position.y",    "position.z",   "orientation.x",
                                                  "orientation.y", "orientation.z", "orientation.w" };

  expected_command_interface_names_.reserve(exp_size_);
  for (const std::string& interface : interface_names)
  {
    expected_command_interface_names_.emplace_back(device_name_ + "/" + interface);
  }

  command_interfaces_.reserve(exp_size_);
  loaned_command_interfaces_.reserve(exp_size_);

  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    command_interfaces_.emplace_back(std::make_shared<hardware_interface::CommandInterface>(device_name_, interface_names[i], &pose_values_[i]));
    loaned_command_interfaces_.emplace_back(*command_interfaces_[i]);
  }
}

/**
 * @brief Test the constructor of the PoseCommand class with wrong size of interface names
 */
TEST_F(PoseCommandTest, test_wrong_size_constructor)
{
  // Test wrong size of interface names
  EXPECT_THROW(acg_semantic_components::PoseCommand(device_name_, std::vector<std::string>{ "wrong_size" }), std::invalid_argument);
}

/**
 * @brief Test the PoseCommand class with custom interface names
 */
TEST_F(PoseCommandTest, test_custom_interface_names)
{
  const std::vector<std::string> custom_interface_names{ "custom_position.x",    "custom_position.y",    "custom_position.z",
                                                         "custom_orientation.x", "custom_orientation.y", "custom_orientation.z",
                                                         "custom_orientation.w" };

  // Create device with custom interface names
  pose_command_ = std::make_unique<acg_semantic_components::PoseCommand>(device_name_, custom_interface_names);

  // Get the interface names
  interface_names_ = pose_command_->get_command_interface_names();

  // Validate correct assignment of interface names
  EXPECT_EQ(interface_names_.size(), custom_interface_names.size());
  EXPECT_TRUE(std::equal(interface_names_.cbegin(), interface_names_.cend(), custom_interface_names.cbegin(), custom_interface_names.cend()));
}

/**
 * @brief Test the PoseCommand class with the default interface names and validate all the functionalities
 */
TEST_F(ConfiguredPoseCommandTest, validate_all)
{
  // Get the interface names
  interface_names_ = pose_command_->get_command_interface_names();

  // Validate the interface names
  ASSERT_EQ(interface_names_.size(), expected_command_interface_names_.size());
  ASSERT_TRUE(std::equal(interface_names_.cbegin(), interface_names_.cend(), expected_command_interface_names_.cbegin(),
                         expected_command_interface_names_.cend()));

  // Assign interfaces
  ASSERT_TRUE(pose_command_->assign_loaned_command_interfaces(loaned_command_interfaces_));

  // Create a message with the expected values
  geometry_msgs::msg::Pose test_pose_msg;
  test_pose_msg.position.x = exp_values_[0];
  test_pose_msg.position.y = exp_values_[1];
  test_pose_msg.position.z = exp_values_[2];
  test_pose_msg.orientation.x = exp_values_[3];
  test_pose_msg.orientation.y = exp_values_[4];
  test_pose_msg.orientation.z = exp_values_[5];
  test_pose_msg.orientation.w = exp_values_[6];

  // Set values of the command interfaces
  ASSERT_TRUE(pose_command_->set_values_from_message(test_pose_msg));

  // Clear the values
  test_pose_msg = geometry_msgs::msg::Pose();

  // Validate correct assignment of values of the command interfaces
  ASSERT_TRUE(pose_command_->get_values_as_message(test_pose_msg));

  // Validate correct assignment of values of the command interfaces
  for (std::size_t i = 0; i < exp_size_; ++i)
  {
    EXPECT_EQ(pose_values_[i], exp_values_[i]);
  }

  // Release command interfaces
  pose_command_->release_interfaces();
}

/**
 * @brief Test the get function of the PoseCommand class when a wrong size input is given to the get_values_as_message function
 */
TEST_F(ConfiguredPoseCommandTest, test_wrong_size_get_values)
{
  // Create the command interfaces
  hardware_interface::CommandInterface position_x{ device_name_, "position.x", &pose_values_[0] };

  // Create a wrong size of loaned command interfaces
  std::vector<hardware_interface::LoanedCommandInterface> wrong_size_loan_command_interfaces;
  wrong_size_loan_command_interfaces.emplace_back(position_x);

  // Assign interfaces
  EXPECT_FALSE(pose_command_->assign_loaned_command_interfaces(wrong_size_loan_command_interfaces));

  geometry_msgs::msg::Pose test_pose_msg;
  EXPECT_FALSE(pose_command_->get_values_as_message(test_pose_msg));

  // Release command interfaces
  pose_command_->release_interfaces();
}

/**
 * @brief Test the set function of the PoseCommand class when a wrong size input is given to the set_values_from_message function
 */
TEST_F(ConfiguredPoseCommandTest, test_wrong_size_set_values)
{
  // Create a few command interfaces
  hardware_interface::CommandInterface position_x{ device_name_, "position.x", &pose_values_[0] };
  hardware_interface::CommandInterface position_y{ device_name_, "position.y", &pose_values_[1] };
  hardware_interface::CommandInterface position_z{ device_name_, "position.z", &pose_values_[2] };

  // Create a wrong size of loaned command interfaces
  std::vector<hardware_interface::LoanedCommandInterface> wrong_size_loan_command_interfaces;
  wrong_size_loan_command_interfaces.emplace_back(position_x);
  wrong_size_loan_command_interfaces.emplace_back(position_y);
  wrong_size_loan_command_interfaces.emplace_back(position_z);

  EXPECT_FALSE(pose_command_->assign_loaned_command_interfaces(wrong_size_loan_command_interfaces));
  geometry_msgs::msg::Pose test_pose_msg;
  test_pose_msg.position.x = exp_values_[0];
  test_pose_msg.position.y = exp_values_[1];
  test_pose_msg.position.z = exp_values_[2];
  test_pose_msg.orientation.x = exp_values_[3];
  test_pose_msg.orientation.y = exp_values_[4];
  test_pose_msg.orientation.z = exp_values_[5];
  test_pose_msg.orientation.w = exp_values_[6];
  EXPECT_FALSE(pose_command_->set_values_from_message(test_pose_msg));

  // Release command interfaces
  pose_command_->release_interfaces();
}
