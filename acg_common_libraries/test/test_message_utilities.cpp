/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_message_utilities.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Apr 2, 2025
 *
 * This is a test for the message_utilities library.
 *
 * -------------------------------------------------------------------
 */

// Gtest
#include <gtest/gtest.h>

// Standard libraries
#include <limits>
#include <cmath>

// Eigen
#include <Eigen/Dense>

// ROS2
#include <geometry_msgs/msg/wrench.hpp>

// Utilities
#include <acg_control_msgs/msg/task_space_point.hpp>
#include <acg_control_msgs/msg/joint_wrench_point.hpp>

// Class under tests
#include "acg_common_libraries/message_utilities.hpp"

/**
 * @brief Test the fromMsg function of the message_utilities library
 */
TEST(MessageUtilitiesTest, TestFromMsg)
{
  // Initialize the wrench
  geometry_msgs::msg::Wrench wrench;
  wrench.force.x = 1.0;
  wrench.force.y = 2.0;
  wrench.force.z = 3.0;
  wrench.torque.x = 4.0;
  wrench.torque.y = 5.0;
  wrench.torque.z = 6.0;

  // Initialize the expected Eigen matrix
  Eigen::Matrix<double, 6, 1> expected_matrix;
  expected_matrix << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  // Convert the wrench to Eigen matrix
  Eigen::Matrix<double, 6, 1> result_matrix;
  tf2::fromMsg(wrench, result_matrix);

  // Check the result
  EXPECT_EQ(result_matrix, expected_matrix);
}

TEST(MessageUtilitiesTest, TestClearWrench)
{
  // Initialize the wrench
  geometry_msgs::msg::Wrench wrench;
  wrench.force.x = 1.0;
  wrench.force.y = 2.0;
  wrench.force.z = 3.0;
  wrench.torque.x = 4.0;
  wrench.torque.y = 5.0;
  wrench.torque.z = 6.0;

  // Clear the wrench
  acg_message_utilities::clear(wrench);

  // Check the wrench
  EXPECT_TRUE(std::isnan(wrench.force.x));
  EXPECT_TRUE(std::isnan(wrench.force.y));
  EXPECT_TRUE(std::isnan(wrench.force.z));
  EXPECT_TRUE(std::isnan(wrench.torque.x));
  EXPECT_TRUE(std::isnan(wrench.torque.y));
  EXPECT_TRUE(std::isnan(wrench.torque.z));
}

/**
 * @brief Test the clear function.
 */
TEST(MessageUtilitiesTest, TestClearTaskSpacePoint)
{
  // Initialize the task space point
  acg_control_msgs::msg::TaskSpacePoint task_space_point;
  task_space_point.pose.position.x = 1.0;
  task_space_point.pose.position.y = 2.0;
  task_space_point.pose.position.z = 3.0;
  task_space_point.pose.orientation.x = 4.0;
  task_space_point.pose.orientation.y = 5.0;
  task_space_point.pose.orientation.z = 6.0;
  task_space_point.twist.linear.x = 7.0;
  task_space_point.twist.linear.y = 8.0;
  task_space_point.twist.linear.z = 9.0;
  task_space_point.twist.angular.x = 10.0;
  task_space_point.twist.angular.y = 11.0;
  task_space_point.twist.angular.z = 12.0;

  // Clear the task space point
  acg_message_utilities::clear(task_space_point);

  // Check the task space point
  EXPECT_TRUE(std::isnan(task_space_point.pose.position.x));
  EXPECT_TRUE(std::isnan(task_space_point.pose.position.y));
  EXPECT_TRUE(std::isnan(task_space_point.pose.position.z));
  EXPECT_TRUE(std::isnan(task_space_point.pose.orientation.x));
  EXPECT_TRUE(std::isnan(task_space_point.pose.orientation.y));
  EXPECT_TRUE(std::isnan(task_space_point.pose.orientation.z));
  EXPECT_TRUE(std::isnan(task_space_point.pose.orientation.w));
  EXPECT_TRUE(std::isnan(task_space_point.twist.linear.x));
  EXPECT_TRUE(std::isnan(task_space_point.twist.linear.y));
  EXPECT_TRUE(std::isnan(task_space_point.twist.linear.z));
  EXPECT_TRUE(std::isnan(task_space_point.twist.angular.x));
  EXPECT_TRUE(std::isnan(task_space_point.twist.angular.y));
  EXPECT_TRUE(std::isnan(task_space_point.twist.angular.z));
  EXPECT_TRUE(std::isnan(task_space_point.acceleration.linear.x));
  EXPECT_TRUE(std::isnan(task_space_point.acceleration.linear.y));
  EXPECT_TRUE(std::isnan(task_space_point.acceleration.linear.z));
  EXPECT_TRUE(std::isnan(task_space_point.acceleration.angular.x));
  EXPECT_TRUE(std::isnan(task_space_point.acceleration.angular.y));
  EXPECT_TRUE(std::isnan(task_space_point.acceleration.angular.z));
  EXPECT_TRUE(std::isnan(task_space_point.wrench.force.x));
  EXPECT_TRUE(std::isnan(task_space_point.wrench.force.y));
  EXPECT_TRUE(std::isnan(task_space_point.wrench.force.z));
  EXPECT_TRUE(std::isnan(task_space_point.wrench.torque.x));
  EXPECT_TRUE(std::isnan(task_space_point.wrench.torque.y));
  EXPECT_TRUE(std::isnan(task_space_point.wrench.torque.z));
  EXPECT_TRUE(std::isnan(task_space_point.wrench_derivative.force.x));
  EXPECT_TRUE(std::isnan(task_space_point.wrench_derivative.force.y));
  EXPECT_TRUE(std::isnan(task_space_point.wrench_derivative.force.z));
  EXPECT_TRUE(std::isnan(task_space_point.wrench_derivative.torque.x));
  EXPECT_TRUE(std::isnan(task_space_point.wrench_derivative.torque.y));
  EXPECT_TRUE(std::isnan(task_space_point.wrench_derivative.torque.z));
}

/**
 * @brief Test the clear function.
 */
TEST(MessageUtilitiesTest, TestClearJointSpacePoint)
{
  // Initialize the joint space point
  acg_control_msgs::msg::JointWrenchPoint joint_space_point;
  joint_space_point.positions.assign({ 1.0, 2.0, 3.0 });
  joint_space_point.velocities.assign({ 4.0, 5.0, 6.0 });
  joint_space_point.accelerations.assign({ 7.0, 8.0, 9.0 });
  joint_space_point.effort.assign({ 10.0, 11.0, 12.0 });
  joint_space_point.wrench.force.x = 4.0;
  joint_space_point.wrench.force.y = 5.0;
  joint_space_point.wrench.force.z = 6.0;
  joint_space_point.wrench.torque.x = 7.0;
  joint_space_point.wrench.torque.y = 8.0;
  joint_space_point.wrench.torque.z = 9.0;

  // Clear the joint space point
  acg_message_utilities::clear(joint_space_point);

  // Check the joint space point
  for (std::size_t i = 0; i < joint_space_point.positions.size(); ++i)
  {
    EXPECT_TRUE(std::isnan(joint_space_point.positions[i]));
  }
  for (std::size_t i = 0; i < joint_space_point.velocities.size(); ++i)
  {
    EXPECT_TRUE(std::isnan(joint_space_point.velocities[i]));
  }
  for (std::size_t i = 0; i < joint_space_point.accelerations.size(); ++i)
  {
    EXPECT_TRUE(std::isnan(joint_space_point.accelerations[i]));
  }
  for (std::size_t i = 0; i < joint_space_point.effort.size(); ++i)
  {
    EXPECT_TRUE(std::isnan(joint_space_point.effort[i]));
  }
  EXPECT_TRUE(std::isnan(joint_space_point.wrench.force.x));
  EXPECT_TRUE(std::isnan(joint_space_point.wrench.force.y));
  EXPECT_TRUE(std::isnan(joint_space_point.wrench.force.z));
  EXPECT_TRUE(std::isnan(joint_space_point.wrench.torque.x));
  EXPECT_TRUE(std::isnan(joint_space_point.wrench.torque.y));
  EXPECT_TRUE(std::isnan(joint_space_point.wrench.torque.z));
}

/**
 * @brief Test the is_nan function when the pose is not NaN.
 */
TEST(MessageUtilitiesTest, TestIsNanWhenPoseIsNotNan)
{
  // Initialize the pose
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.position.z = 3.0;
  pose.orientation.x = 4.0;
  pose.orientation.y = 5.0;
  pose.orientation.z = 6.0;
  pose.orientation.w = 7.0;

  // Check if the pose is NaN
  EXPECT_FALSE(acg_message_utilities::is_nan(pose));
}

/**
 * @brief Test the is_nan function when the pose is NaN.
 */
TEST(MessageUtilitiesTest, TestIsNanWhenPoseIsNan)
{
  // Initialize the pose with default values
  geometry_msgs::msg::Pose pose;

  // Set at least one element of NaN
  pose.position.x = std::numeric_limits<double>::quiet_NaN();

  // Check if the pose is NaN
  EXPECT_TRUE(acg_message_utilities::is_nan(pose));
}

/**
 * @brief Test the is_nan function when the twist is not NaN.
 */
TEST(MessageUtilitiesTest, TestIsNanWhenTwistIsNotNan)
{
  // Initialize the twist
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0;
  twist.linear.y = 2.0;
  twist.linear.z = 3.0;
  twist.angular.x = 4.0;
  twist.angular.y = 5.0;
  twist.angular.z = 6.0;

  // Check if the twist is NaN
  EXPECT_FALSE(acg_message_utilities::is_nan(twist));
}

/**
 * @brief Test the is_nan function when the twist is NaN.
 */
TEST(MessageUtilitiesTest, TestIsNanWhenTwistIsNan)
{
  // Initialize the twist with default values
  geometry_msgs::msg::Twist twist;

  // Set at least one element of the twist to NaN
  twist.linear.x = std::numeric_limits<double>::quiet_NaN();

  // Check if the twist is NaN
  EXPECT_TRUE(acg_message_utilities::is_nan(twist));
}

/**
 * @brief Test the is_valid function when the twist is valid.
 */
TEST(MessageUtilitiesTest, TestIsValidWhenTwistIsValid)
{
  // Initialize the twist
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0;
  twist.linear.y = 2.0;
  twist.linear.z = 3.0;
  twist.angular.x = 4.0;
  twist.angular.y = 5.0;
  twist.angular.z = 6.0;

  // Check if the twist is valid
  EXPECT_TRUE(acg_message_utilities::is_valid(twist));
}

/**
 * @brief Test the is_valid function when the twist is not valid.
 */
TEST(MessageUtilitiesTest, TestIsValidWhenTwistIsNotValid)
{
  // Initialize the twist with default values
  geometry_msgs::msg::Twist twist;

  // Set at least one element of the twist to NaN
  twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  twist.linear.y = 2.0;
  twist.linear.z = 3.0;
  twist.angular.x = 4.0;
  twist.angular.y = 5.0;
  twist.angular.z = 6.0;

  // Check if the twist is valid
  EXPECT_FALSE(acg_message_utilities::is_valid(twist));
}
