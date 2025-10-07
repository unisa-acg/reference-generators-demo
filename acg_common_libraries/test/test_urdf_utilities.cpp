/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_urdf_utilities.cpp
 * Author:  Antonio Langella, Davide Risi
 * Org.:    UNISA
 * Date:    May 30, 2025
 *
 * This is a test for the urdf_utilities library.
 *
 * -------------------------------------------------------------------
 */

// Gtest
#include <gtest/gtest.h>

// Mock robot description
#include "ros2_control_test_assets/descriptions.hpp"

// Class under tests
#include "acg_common_libraries/urdf_utilities.hpp"

// This class implements the tests
class URDFUtilitiesTests : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Build the full URDF model from the strings
    robot_description_string = ros2_control_test_assets::urdf_head;
    robot_description_string += ros2_control_test_assets::urdf_tail;

    // Load the URDF model
    ASSERT_TRUE(robot_urdf.initString(robot_description_string));

    // Set the root link pointer
    root_link = robot_urdf.getLink("base_link");
  }

public:
  std::string robot_description_string;
  urdf::Model robot_urdf;
  urdf::LinkConstSharedPtr root_link;
};

/**
 * @brief Tests the compute_chain_mass function of the \c urdf_utilities library.
 *
 * This test verifies that the function correctly calculates the total mass of a chain
 * of links in a robot URDF model starting from a specified link.
 */
TEST_F(URDFUtilitiesTests, ComputeChainMassTest)
{
  // Expected mass of the chain (0.01 + 0.01 + 0.01)
  const double expected_mass = 0.03;

  // Calculate the mass of the chain starting from the root link
  double mass = urdf_utilities::compute_chain_mass(root_link);

  // Check if the computed mass matches the expected value
  EXPECT_EQ(mass, expected_mass);
}

/**
 * @brief Test the compute_chain_mass_and_com function of the \c urdf_utilities library.
 *
 * This test checks if the function can compute the mass and center of mass (COM) of a chain of
 * links in a robot URDF model starting from a specified link.
 */
TEST_F(URDFUtilitiesTests, ComputeChainCOMTest)
{
  // Expected mass of the chain (0.01 + 0.01 + 0.01)
  const double expected_mass = 0.03;

  // Expected center of mass (COM) position
  const Eigen::Vector3d expected_com(0.0, 0.3, 0.133333);

  // Calculate the total mass and center of mass of the chain starting from the root link
  auto [mass, com] = urdf_utilities::compute_chain_mass_and_com(root_link);

  // Check if the computed mass and COM match the expected values
  EXPECT_NEAR(mass, expected_mass, 1e-6);
  EXPECT_NEAR(com.x(), expected_com.x(), 1e-6);
  EXPECT_NEAR(com.y(), expected_com.y(), 1e-6);
  EXPECT_NEAR(com.z(), expected_com.z(), 1e-6);
}

/**
 * @brief Tests the retrieval of joint position limits from a URDF model.
 *
 * This test verifies that the function correctly retrieves the position limits for specified joints
 * from the URDF model and compares them with the expected values.
 *
 */
TEST_F(URDFUtilitiesTests, RetrieveJointPositionLimitsTest)
{
  const std::vector<double> expected_joint_upper_limits = { 3.14159265359, 3.14159265359 };
  const std::vector<double> expected_joint_lower_limits = { -3.14159265359, -3.14159265359 };
  std::vector<double> joint_upper_limits;
  std::vector<double> joint_lower_limits;
  std::vector<std::string> joint_names = { "joint1", "joint2" };

  // Retrieve the joint position limits for all the joints in the URDF model
  urdf_utilities::read_joint_position_limits_from_urdf(robot_description_string, joint_names, joint_upper_limits, joint_lower_limits);

  ASSERT_EQ(joint_upper_limits.size(), expected_joint_upper_limits.size());
  ASSERT_EQ(joint_lower_limits.size(), expected_joint_lower_limits.size());

  for (size_t i = 0; i < joint_upper_limits.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(joint_upper_limits[i], expected_joint_upper_limits[i]);
    EXPECT_DOUBLE_EQ(joint_lower_limits[i], expected_joint_lower_limits[i]);
  }
}

/**
 * @brief Tests the retrieval of joint velocity limits from a URDF model.
 *
 * This test verifies that the function correctly retrieves the velocity limits for specified joints
 * from the URDF model and compares them with the expected values.
 */
TEST_F(URDFUtilitiesTests, RetrieveJointVelocityLimitsTest)
{
  const std::vector<double> expected_joint_velocity_limits = { 0.2, 0.2 };
  std::vector<double> joint_velocity_limits;
  std::vector<std::string> joint_names = { "joint1", "joint2" };

  // Retrieve the joint velocity limits for all the joints in the URDF model
  urdf_utilities::read_joint_velocity_limits_from_urdf(robot_description_string, joint_names, joint_velocity_limits);

  ASSERT_EQ(joint_velocity_limits.size(), expected_joint_velocity_limits.size());
  for (size_t i = 0; i < joint_velocity_limits.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(joint_velocity_limits[i], expected_joint_velocity_limits[i]);
  }
}

/**
 * @brief Tests the retrieval of joint effort limits from a URDF model.
 *
 * This test verifies that the function correctly retrieves the effort limits for specified joints
 * from the URDF model and compares them with the expected values.
 */
TEST_F(URDFUtilitiesTests, RetrieveJointEffortLimitsTest)
{
  const std::vector<double> expected_joint_effort_limits = { 0.1, 0.1 };
  std::vector<double> joint_effort_limits;
  std::vector<std::string> joint_names = { "joint1", "joint2" };

  // Retrieve the joint effort limits for all the joints in the URDF model
  urdf_utilities::read_joint_effort_limits_from_urdf(robot_description_string, joint_names, joint_effort_limits);

  ASSERT_EQ(joint_effort_limits.size(), expected_joint_effort_limits.size());
  for (size_t i = 0; i < joint_effort_limits.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(joint_effort_limits[i], expected_joint_effort_limits[i]);
  }
}

/**
 * @brief Tests the behavior of the URDF utilities when joint limits are not defined.
 */
TEST_F(URDFUtilitiesTests, JointLimitsNotDefinedExceptionTest)
{
  std::vector<std::string> joint_names = { "base_joint" };  // This joint is fixed and does not have limits defined
  joint_names.push_back("joint1");                          // Adding a valid joint to ensure the test is comprehensive

  std::vector<double> joint_upper_limits;
  std::vector<double> joint_lower_limits;
  std::vector<double> joint_velocity_limits;
  std::vector<double> joint_effort_limits;

  std::vector<double> expected_joint_upper_limits = { std::numeric_limits<double>::max(), 3.14159265359 };
  std::vector<double> expected_joint_lower_limits = { std::numeric_limits<double>::lowest(), -3.14159265359 };
  std::vector<double> expected_joint_velocity_limits = { std::numeric_limits<double>::max(), 0.2 };
  std::vector<double> expected_joint_effort_limits = { std::numeric_limits<double>::max(), 0.1 };

  // Expect an exception to be thrown when trying to read limits for a joint without defined limits
  EXPECT_THROW(urdf_utilities::read_joint_position_limits_from_urdf(robot_description_string, joint_names, joint_upper_limits, joint_lower_limits),
               urdf_utilities::JointLimitsNotDefinedException);
  EXPECT_THROW(urdf_utilities::read_joint_velocity_limits_from_urdf(robot_description_string, joint_names, joint_velocity_limits),
               urdf_utilities::JointLimitsNotDefinedException);
  EXPECT_THROW(urdf_utilities::read_joint_effort_limits_from_urdf(robot_description_string, joint_names, joint_effort_limits),
               urdf_utilities::JointLimitsNotDefinedException);

  // Check if the limits for the valid joint are correctly set
  ASSERT_EQ(joint_upper_limits.size(), expected_joint_upper_limits.size());
  ASSERT_EQ(joint_lower_limits.size(), expected_joint_lower_limits.size());
  ASSERT_EQ(joint_velocity_limits.size(), expected_joint_velocity_limits.size());
  ASSERT_EQ(joint_effort_limits.size(), expected_joint_effort_limits.size());

  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    EXPECT_EQ(joint_upper_limits[i], expected_joint_upper_limits[i]);
    EXPECT_EQ(joint_lower_limits[i], expected_joint_lower_limits[i]);
    EXPECT_EQ(joint_velocity_limits[i], expected_joint_velocity_limits[i]);
    EXPECT_EQ(joint_effort_limits[i], expected_joint_effort_limits[i]);
  }
}

/**
 * @brief Tests the behavior of the URDF utilities when a joint is not found in the URDF.
 */
TEST_F(URDFUtilitiesTests, JointNotFoundExceptionTest)
{
  std::vector<std::string> joint_names = { "non_existent_joint" };  // This joint does not exist in the URDF
  std::vector<double> joint_upper_limits;
  std::vector<double> joint_lower_limits;

  // Expect an exception to be thrown when trying to read limits for a non-existent joint
  EXPECT_THROW(urdf_utilities::read_joint_position_limits_from_urdf(robot_description_string, joint_names, joint_upper_limits, joint_lower_limits),
               std::runtime_error);
  EXPECT_THROW(urdf_utilities::read_joint_velocity_limits_from_urdf(robot_description_string, joint_names, joint_upper_limits), std::runtime_error);
  EXPECT_THROW(urdf_utilities::read_joint_effort_limits_from_urdf(robot_description_string, joint_names, joint_upper_limits), std::runtime_error);
}
