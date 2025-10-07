/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_interpolation.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Mar 29, 2025
 *
 * This is a test for the interpolation library.
 *
 * -------------------------------------------------------------------
 */

// Gtest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Dense>

// ROS2
#include <rclcpp/rclcpp.hpp>

// Utilities
#include <acg_control_msgs/msg/joint_wrench_point.hpp>
#include <acg_control_msgs/msg/joint_trajectory_point.hpp>
#include <acg_control_msgs/msg/task_space_point.hpp>
#include <acg_control_msgs/msg/task_space_trajectory_point.hpp>

// File under tests
#include "acg_common_libraries/interpolation.hpp"

// ****** Definitions of the test parameters and test cases for std::vector<double> interpolation ****** //

struct VectorDoubleTestParameters
{
  std::vector<double> first_point;
  std::vector<double> second_point;
  double t_normalized;
  std::vector<double> expected_point;
};

class VectorDoubleInterpolationTestSuite : public testing::TestWithParam<VectorDoubleTestParameters>
{
public:
  void SetUp() override
  {
    const VectorDoubleTestParameters params = GetParam();
    first_point = params.first_point;
    second_point = params.second_point;
    t_normalized = params.t_normalized;
    expected_point = params.expected_point;
    interpolated_point.resize(first_point.size(), std::numeric_limits<double>::quiet_NaN());
  }

protected:
  std::vector<double> first_point;
  std::vector<double> second_point;
  double t_normalized;
  std::vector<double> expected_point;
  std::vector<double> interpolated_point;
};

/**
 * @brief Test the \c linearly_interpolate function of the \c interpolation library on std::vector<double>
 */
TEST_P(VectorDoubleInterpolationTestSuite, LinearlyInterpolate)
{
  acg_interpolation::linearly_interpolate(first_point, second_point, t_normalized, interpolated_point);
  EXPECT_EQ(interpolated_point, expected_point);
}

// Helper functions to create test parameters
VectorDoubleTestParameters createVectorDoubleNormalCase()
{
  VectorDoubleTestParameters params;
  params.first_point = { 0.0, 0.0, 0.0 };
  params.second_point = { 1.0, 1.0, 1.0 };
  params.t_normalized = 0.5;
  params.expected_point = { 0.5, 0.5, 0.5 };
  return params;
}

VectorDoubleTestParameters createVectorDoubleEdgeCaseTimeZero()
{
  VectorDoubleTestParameters params;
  params.first_point = { 1.0, 2.0, 3.0 };
  params.second_point = { 4.0, 5.0, 6.0 };
  params.t_normalized = 0.0;
  params.expected_point = { 1.0, 2.0, 3.0 };
  return params;
}

VectorDoubleTestParameters createVectorDoubleEdgeCaseTimeOne()
{
  VectorDoubleTestParameters params;
  params.first_point = { 1.0, 2.0, 3.0 };
  params.second_point = { 4.0, 5.0, 6.0 };
  params.t_normalized = 1.0;
  params.expected_point = { 4.0, 5.0, 6.0 };
  return params;
}

VectorDoubleTestParameters createVectorDoubleEdgeCaseSamePoints()
{
  VectorDoubleTestParameters params;
  params.first_point = { 2.0, 2.0, 2.0 };
  params.second_point = { 2.0, 2.0, 2.0 };
  params.t_normalized = 0.7;
  params.expected_point = { 2.0, 2.0, 2.0 };
  return params;
}

INSTANTIATE_TEST_SUITE_P(VectorDoubleInterpolationTests, VectorDoubleInterpolationTestSuite,
                         testing::Values(createVectorDoubleNormalCase(), createVectorDoubleEdgeCaseTimeZero(), createVectorDoubleEdgeCaseTimeOne(),
                                         createVectorDoubleEdgeCaseSamePoints()));
/**
 * @brief Test that the \c linearly_interpolate function of the \c interpolation library throws an exception when the input sizes are different.
 */
TEST(VectorDoubleInterpolationExceptionTest, ThrowsOnDifferentInputSizes)
{
  std::vector<double> a{ 1.0, 2.0 };  // Wrong size
  std::vector<double> b{ 1.0, 2.0, 3.0 };
  std::vector<double> out(2);
  EXPECT_THROW(acg_interpolation::linearly_interpolate(a, b, 0.5, out), std::runtime_error);
}

/**
 * @brief Test that the \c linearly_interpolate function of the \c interpolation library throws an exception when the output size is different from
 * the input sizes.
 */
TEST(VectorDoubleInterpolationExceptionTest, ThrowsOnWrongOutputSize)
{
  std::vector<double> a{ 1.0, 2.0, 3.0 };
  std::vector<double> b{ 4.0, 5.0, 6.0 };
  std::vector<double> out(2);  // Wrong size
  EXPECT_THROW(acg_interpolation::linearly_interpolate(a, b, 0.5, out), std::runtime_error);
}

// ****** Definitions of the test cases for static Eigen::Matrix interpolation ****** //
template <typename MatrixType>
class StaticMatrixInterpolationTestSuite : public testing::Test
{};

using StaticMatrixInterpolationTestTypes =
    ::testing::Types<Eigen::Matrix2d, Eigen::Matrix3d, Eigen::Matrix4d, Eigen::Vector2d, Eigen::Vector3d, Eigen::Vector4d>;
TYPED_TEST_SUITE(StaticMatrixInterpolationTestSuite, StaticMatrixInterpolationTestTypes);

/**
 * @brief Test the \c linearly_interpolate function of the \c interpolation library on static Eigen matrices. Normal case.
 */
TYPED_TEST(StaticMatrixInterpolationTestSuite, NormalCase)
{
  TypeParam first_point = TypeParam::Zero();
  TypeParam second_point = TypeParam::Ones();
  double t_normalized = 0.5;
  TypeParam expected_point = TypeParam::Constant(0.5);
  TypeParam interpolated_point;
  acg_interpolation::linearly_interpolate(first_point, second_point, t_normalized, interpolated_point);
  EXPECT_EQ(interpolated_point, expected_point);
}

/**
 * @brief Test the \c linearly_interpolate function of the \c interpolation library on static Eigen matrices. Edge case: time is zero.
 */
TYPED_TEST(StaticMatrixInterpolationTestSuite, EdgeCaseTimeZero)
{
  TypeParam first_point = TypeParam::Zero();
  TypeParam second_point = TypeParam::Ones();
  double t_normalized = 0.0;
  TypeParam expected_point = first_point;
  TypeParam interpolated_point;
  acg_interpolation::linearly_interpolate(first_point, second_point, t_normalized, interpolated_point);
  EXPECT_EQ(interpolated_point, expected_point);
}

/**
 * @brief Test the \c linearly_interpolate function of the \c interpolation library on static Eigen matrices. Edge case: time is one.
 */
TYPED_TEST(StaticMatrixInterpolationTestSuite, EdgeCaseTimeOne)
{
  TypeParam first_point = TypeParam::Zero();
  TypeParam second_point = TypeParam::Ones();
  double t_normalized = 1.0;
  TypeParam expected_point = second_point;
  TypeParam interpolated_point;
  acg_interpolation::linearly_interpolate(first_point, second_point, t_normalized, interpolated_point);
  EXPECT_EQ(interpolated_point, expected_point);
}

/**
 * @brief Test the \c linearly_interpolate function of the \c interpolation library on static Eigen matrices. Edge case: same points.
 */
TYPED_TEST(StaticMatrixInterpolationTestSuite, EdgeCaseSamePoints)
{
  TypeParam first_point = TypeParam::Zero();
  TypeParam second_point = TypeParam::Zero();
  double t_normalized = 0.75;
  TypeParam interpolated_point;
  acg_interpolation::linearly_interpolate(first_point, second_point, t_normalized, interpolated_point);
  EXPECT_EQ(interpolated_point, first_point);
  EXPECT_EQ(interpolated_point, second_point);
}

// ****** Definitions of the test parameters and test cases for dynamic Eigen::Matrix interpolation ****** //
using DynamicMatrix = Eigen::MatrixXd;

// Define the parameters for DynamicMatrixInterpolationTestSuite
struct DynamicMatrixInterpolationTestParameters
{
  DynamicMatrix first_point;
  DynamicMatrix second_point;
  double t_normalized;
  DynamicMatrix expected_point;
};

class DynamicMatrixInterpolationTestSuite : public testing::TestWithParam<DynamicMatrixInterpolationTestParameters>
{
public:
  void SetUp() override
  {
    const DynamicMatrixInterpolationTestParameters params = GetParam();
    first_point = params.first_point;
    second_point = params.second_point;
    t_normalized = params.t_normalized;
    expected_point = params.expected_point;
  }

protected:
  DynamicMatrix first_point;
  DynamicMatrix second_point;
  double t_normalized;
  DynamicMatrix expected_point;
  DynamicMatrix interpolated_point;
};

/**
 * @brief Test the \c linearly_interpolate function of the \c interpolation library on dynamic Eigen matrices
 */
TEST_P(DynamicMatrixInterpolationTestSuite, LinearlyInterpolate)
{
  acg_interpolation::linearly_interpolate(first_point, second_point, t_normalized, interpolated_point);
  EXPECT_EQ(interpolated_point, expected_point);
}

// Helper functions to create DynamicMatrixInterpolationTestParameters for different test cases
DynamicMatrixInterpolationTestParameters createDynamicMatrixInterpolationNormalCase1()
{
  DynamicMatrixInterpolationTestParameters params;
  params.first_point = DynamicMatrix::Zero(3, 3);
  params.second_point = DynamicMatrix::Ones(3, 3);
  params.t_normalized = 0.5;
  params.expected_point = DynamicMatrix::Constant(3, 3, 0.5);
  return params;
}

DynamicMatrixInterpolationTestParameters createDynamicMatrixInterpolationNormalCase2()
{
  DynamicMatrixInterpolationTestParameters params;
  params.first_point = DynamicMatrix::Zero(4, 4);
  params.second_point = DynamicMatrix::Ones(4, 4);
  params.t_normalized = 0.5;
  params.expected_point = DynamicMatrix::Constant(4, 4, 0.5);
  return params;
}

DynamicMatrixInterpolationTestParameters createDynamicMatrixInterpolationNormalCase3()
{
  DynamicMatrixInterpolationTestParameters params;
  params.first_point = DynamicMatrix::Zero(2, 2);
  params.second_point = DynamicMatrix::Ones(2, 2);
  params.t_normalized = 0.5;
  params.expected_point = DynamicMatrix::Constant(2, 2, 0.5);
  return params;
}

DynamicMatrixInterpolationTestParameters createDynamicMatrixInterpolationEdgeCaseSamePoints()
{
  DynamicMatrixInterpolationTestParameters params;
  params.first_point = DynamicMatrix::Zero(3, 3);
  params.second_point = DynamicMatrix::Zero(3, 3);
  params.t_normalized = 0.5;
  params.expected_point = DynamicMatrix::Zero(3, 3);
  return params;
}

DynamicMatrixInterpolationTestParameters createDynamicMatrixInterpolationEdgeCaseTimeZero()
{
  DynamicMatrixInterpolationTestParameters params;
  params.first_point = DynamicMatrix::Zero(3, 3);
  params.second_point = DynamicMatrix::Ones(3, 3);
  params.t_normalized = 0;
  params.expected_point = DynamicMatrix::Zero(3, 3);
  return params;
}

DynamicMatrixInterpolationTestParameters createDynamicMatrixInterpolationEdgeCaseTimeOne()
{
  DynamicMatrixInterpolationTestParameters params;
  params.first_point = DynamicMatrix::Zero(3, 3);
  params.second_point = DynamicMatrix::Ones(3, 3);
  params.t_normalized = 1;
  params.expected_point = DynamicMatrix::Ones(3, 3);
  return params;
}

INSTANTIATE_TEST_SUITE_P(DynamicMatrixInterpolationTests, DynamicMatrixInterpolationTestSuite,
                         testing::Values(createDynamicMatrixInterpolationNormalCase1(), createDynamicMatrixInterpolationNormalCase2(),
                                         createDynamicMatrixInterpolationNormalCase3(), createDynamicMatrixInterpolationEdgeCaseSamePoints(),
                                         createDynamicMatrixInterpolationEdgeCaseTimeZero(), createDynamicMatrixInterpolationEdgeCaseTimeOne()));

// ****** Definitions of the test parameters and test cases for Eigen::Vector3d interpolation ****** //

using Vector3d = Eigen::Vector3d;

// Define the parameters for Vector3DInterpolationTestSuite
struct Vector3DTestParameters
{
  Vector3d first_point;
  Vector3d second_point;
  double t_normalized;
  Vector3d expected_point;
};

class Vector3DInterpolationTestSuite : public testing::TestWithParam<Vector3DTestParameters>
{
public:
  void SetUp() override
  {
    // Set up the test parameters from the test fixture
    const Vector3DTestParameters params = GetParam();
    first_point = params.first_point;
    second_point = params.second_point;
    t_normalized = params.t_normalized;
    expected_point = params.expected_point;
  }

protected:
  Vector3d first_point;
  Vector3d second_point;
  double t_normalized;
  Vector3d expected_point;
  Vector3d interpolated_point;
};

/**
 * @brief Test the \c linearly_interpolate function of the \c interpolation library on Eigen arrays
 */
TEST_P(Vector3DInterpolationTestSuite, LinearlyInterpolate)
{
  acg_interpolation::linearly_interpolate(first_point, second_point, t_normalized, interpolated_point);
  EXPECT_EQ(interpolated_point, expected_point);
}

// Helper functions to create Vector3DTestParameters for different test cases
Vector3DTestParameters createNormalCase1()
{
  Vector3DTestParameters params;
  params.first_point = Vector3d(0, 0, 0);
  params.second_point = Vector3d(1, 1, 1);
  params.t_normalized = 0.5;
  params.expected_point = Vector3d(0.5, 0.5, 0.5);
  return params;
}

Vector3DTestParameters createNormalCase2()
{
  Vector3DTestParameters params;
  params.first_point = Vector3d(-1, -2, -3);
  params.second_point = Vector3d(4, 5, 6);
  params.t_normalized = 0.5;
  params.expected_point = Vector3d(1.5, 1.5, 1.5);
  return params;
}

Vector3DTestParameters createNormalCase3()
{
  Vector3DTestParameters params;
  params.first_point = Vector3d(2, 2, 2);
  params.second_point = Vector3d(4, 4, 4);
  params.t_normalized = 0.5;
  params.expected_point = Vector3d(3, 3, 3);
  return params;
}

Vector3DTestParameters createEdgeCaseSamePoints()
{
  Vector3DTestParameters params;
  params.first_point = Vector3d(0, 0, 0);
  params.second_point = Vector3d(0, 0, 0);
  params.t_normalized = 0.5;
  params.expected_point = Vector3d(0, 0, 0);
  return params;
}

Vector3DTestParameters createEdgeCaseTimeZero()
{
  Vector3DTestParameters params;
  params.first_point = Vector3d(0, 0, 0);
  params.second_point = Vector3d(1, 1, 1);
  params.t_normalized = 0;
  params.expected_point = Vector3d(0, 0, 0);
  return params;
}

Vector3DTestParameters createEdgeCaseTimeOne()
{
  Vector3DTestParameters params;
  params.first_point = Vector3d(0, 0, 0);
  params.second_point = Vector3d(1, 1, 1);
  params.t_normalized = 1;
  params.expected_point = Vector3d(1, 1, 1);
  return params;
}

INSTANTIATE_TEST_SUITE_P(InterpolationTests, Vector3DInterpolationTestSuite,
                         testing::Values(createNormalCase1(), createNormalCase2(), createNormalCase3(), createEdgeCaseSamePoints(),
                                         createEdgeCaseTimeZero(), createEdgeCaseTimeOne()));

// ****** Definitions of the test parameters and test cases for joint point interpolation ****** //

// Define the parameters for the test
struct JointPointTestParameters
{
  acg_control_msgs::msg::JointTrajectoryPoint first_point;
  acg_control_msgs::msg::JointTrajectoryPoint second_point;
  double t;
  acg_control_msgs::msg::JointWrenchPoint expected_point;
};

class JointPointInterpolationTestSuite : public testing::TestWithParam<JointPointTestParameters>
{
public:
  void SetUp() override
  {
    const JointPointTestParameters params = GetParam();
    first_point = params.first_point;
    second_point = params.second_point;
    t = params.t;
    expected_point = params.expected_point;

    interpolated_point.positions.assign(first_point.point.positions.size(), std::numeric_limits<double>::quiet_NaN());
    interpolated_point.velocities.assign(first_point.point.velocities.size(), std::numeric_limits<double>::quiet_NaN());
    interpolated_point.accelerations.assign(first_point.point.accelerations.size(), std::numeric_limits<double>::quiet_NaN());
    interpolated_point.effort.assign(first_point.point.effort.size(), std::numeric_limits<double>::quiet_NaN());
  }

protected:
  acg_control_msgs::msg::JointTrajectoryPoint first_point;
  acg_control_msgs::msg::JointTrajectoryPoint second_point;
  double t;
  acg_control_msgs::msg::JointWrenchPoint expected_point;
  acg_control_msgs::msg::JointWrenchPoint interpolated_point;
};

/**
 * @brief Test the \c linearly_interpolate function of the \c interpolation library on joint points
 */
TEST_P(JointPointInterpolationTestSuite, SampleJointPoint)
{
  // Call the function under test
  acg_interpolation::linearly_interpolate(first_point, second_point, t, interpolated_point);

  // Validate the results
  EXPECT_EQ(interpolated_point, expected_point);
}

JointPointTestParameters createJointPointNormalCase()
{
  JointPointTestParameters params;
  params.first_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
  params.first_point.point.positions = { 0.0, 0.0, 0.0 };
  params.first_point.point.velocities = { 0.0, 0.0, 0.0 };
  params.first_point.point.accelerations = { 0.0, 0.0, 0.0 };
  params.first_point.point.effort = { 0.0, 0.0, 0.0 };
  params.first_point.point.wrench.force.x = 0.0;
  params.first_point.point.wrench.force.y = 0.0;
  params.first_point.point.wrench.force.z = 0.0;
  params.first_point.point.wrench.torque.x = 0.0;
  params.first_point.point.wrench.torque.y = 0.0;
  params.first_point.point.wrench.torque.z = 0.0;

  params.second_point.time_from_start = rclcpp::Duration::from_seconds(1.0);
  params.second_point.point.positions = { 1.0, 1.0, 1.0 };
  params.second_point.point.velocities = { 1.0, 1.0, 1.0 };
  params.second_point.point.accelerations = { 1.0, 1.0, 1.0 };
  params.second_point.point.effort = { 1.0, 1.0, 1.0 };
  params.second_point.point.wrench.force.x = 1.0;
  params.second_point.point.wrench.force.y = 1.0;
  params.second_point.point.wrench.force.z = 1.0;
  params.second_point.point.wrench.torque.x = 1.0;
  params.second_point.point.wrench.torque.y = 1.0;
  params.second_point.point.wrench.torque.z = 1.0;

  params.t = 0.5;

  params.expected_point.positions = { 0.5, 0.5, 0.5 };
  params.expected_point.velocities = { 0.5, 0.5, 0.5 };
  params.expected_point.accelerations = { 0.5, 0.5, 0.5 };
  params.expected_point.effort = { 0.5, 0.5, 0.5 };
  params.expected_point.wrench.force.x = 0.5;
  params.expected_point.wrench.force.y = 0.5;
  params.expected_point.wrench.force.z = 0.5;
  params.expected_point.wrench.torque.x = 0.5;
  params.expected_point.wrench.torque.y = 0.5;
  params.expected_point.wrench.torque.z = 0.5;

  return params;
}

JointPointTestParameters createJointPointEdgeCaseTZero()
{
  JointPointTestParameters params;

  params.first_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
  params.first_point.point.positions = { 1.0, 1.0, 1.0 };
  params.first_point.point.velocities = { 1.0, 1.0, 1.0 };
  params.first_point.point.accelerations = { 1.0, 1.0, 1.0 };
  params.first_point.point.effort = { 1.0, 1.0, 1.0 };
  params.first_point.point.wrench.force.x = 1.0;
  params.first_point.point.wrench.force.y = 1.0;
  params.first_point.point.wrench.force.z = 1.0;
  params.first_point.point.wrench.torque.x = 1.0;
  params.first_point.point.wrench.torque.y = 1.0;
  params.first_point.point.wrench.torque.z = 1.0;

  params.second_point.time_from_start = rclcpp::Duration::from_seconds(1.0);
  params.second_point.point.positions = { 2.0, 2.0, 2.0 };
  params.second_point.point.velocities = { 2.0, 2.0, 2.0 };
  params.second_point.point.accelerations = { 2.0, 2.0, 2.0 };
  params.second_point.point.effort = { 2.0, 2.0, 2.0 };
  params.second_point.point.wrench.force.x = 2.0;
  params.second_point.point.wrench.force.y = 2.0;
  params.second_point.point.wrench.force.z = 2.0;
  params.second_point.point.wrench.torque.x = 2.0;
  params.second_point.point.wrench.torque.y = 2.0;
  params.second_point.point.wrench.torque.z = 2.0;

  params.t = 0.0;

  params.expected_point.positions = { 1.0, 1.0, 1.0 };
  params.expected_point.velocities = { 1.0, 1.0, 1.0 };
  params.expected_point.accelerations = { 1.0, 1.0, 1.0 };
  params.expected_point.effort = { 1.0, 1.0, 1.0 };
  params.expected_point.wrench.force.x = 1.0;
  params.expected_point.wrench.force.y = 1.0;
  params.expected_point.wrench.force.z = 1.0;
  params.expected_point.wrench.torque.x = 1.0;
  params.expected_point.wrench.torque.y = 1.0;
  params.expected_point.wrench.torque.z = 1.0;

  return params;
}

JointPointTestParameters createJointPointEdgeCaseTOne()
{
  JointPointTestParameters params;

  params.first_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
  params.first_point.point.positions = { 1.0, 1.0, 1.0 };
  params.first_point.point.velocities = { 1.0, 1.0, 1.0 };
  params.first_point.point.accelerations = { 1.0, 1.0, 1.0 };
  params.first_point.point.effort = { 1.0, 1.0, 1.0 };
  params.first_point.point.wrench.force.x = 1.0;
  params.first_point.point.wrench.force.y = 1.0;
  params.first_point.point.wrench.force.z = 1.0;
  params.first_point.point.wrench.torque.x = 1.0;
  params.first_point.point.wrench.torque.y = 1.0;
  params.first_point.point.wrench.torque.z = 1.0;

  params.second_point.time_from_start = rclcpp::Duration::from_seconds(1.0);
  params.second_point.point.positions = { 2.0, 2.0, 2.0 };
  params.second_point.point.velocities = { 2.0, 2.0, 2.0 };
  params.second_point.point.accelerations = { 2.0, 2.0, 2.0 };
  params.second_point.point.effort = { 2.0, 2.0, 2.0 };
  params.second_point.point.wrench.force.x = 2.0;
  params.second_point.point.wrench.force.y = 2.0;
  params.second_point.point.wrench.force.z = 2.0;
  params.second_point.point.wrench.torque.x = 2.0;
  params.second_point.point.wrench.torque.y = 2.0;
  params.second_point.point.wrench.torque.z = 2.0;

  params.t = 1.0;

  params.expected_point.positions = { 2.0, 2.0, 2.0 };
  params.expected_point.velocities = { 2.0, 2.0, 2.0 };
  params.expected_point.accelerations = { 2.0, 2.0, 2.0 };
  params.expected_point.effort = { 2.0, 2.0, 2.0 };
  params.expected_point.wrench.force.x = 2.0;
  params.expected_point.wrench.force.y = 2.0;
  params.expected_point.wrench.force.z = 2.0;
  params.expected_point.wrench.torque.x = 2.0;
  params.expected_point.wrench.torque.y = 2.0;
  params.expected_point.wrench.torque.z = 2.0;

  return params;
}

// Instantiate test suite with test cases
INSTANTIATE_TEST_SUITE_P(JointPointInterpolationTests, JointPointInterpolationTestSuite,
                         testing::Values(createJointPointNormalCase(), createJointPointEdgeCaseTZero(), createJointPointEdgeCaseTOne()));

/**
 * @brief Test that the \c linearly_interpolate function of the \c interpolation on joint points throws an exception when the point timestamps are not
 * in the correct order.
 */
TEST(JointPointInterpolationExceptionTest, ThrowsOnFirstTimestampGreaterOrEqual)
{
  acg_control_msgs::msg::JointTrajectoryPoint first, second;
  first.time_from_start = rclcpp::Duration::from_seconds(2.0);
  second.time_from_start = rclcpp::Duration::from_seconds(1.0);
  acg_control_msgs::msg::JointWrenchPoint out;
  EXPECT_THROW(acg_interpolation::linearly_interpolate(first, second, 1.5, out), std::runtime_error);
}

/**
 * @brief Test that the \c linearly_interpolate function of the \c interpolation library on joint points throws an exception when the provided time is
 * out of bounds.
 */
TEST(JointPointInterpolationExceptionTest, ThrowsOnTimeOutOfBounds)
{
  acg_control_msgs::msg::JointTrajectoryPoint first, second;
  first.time_from_start = rclcpp::Duration::from_seconds(0.0);
  second.time_from_start = rclcpp::Duration::from_seconds(1.0);
  acg_control_msgs::msg::JointWrenchPoint out;
  // t < first_timestamp
  EXPECT_THROW(acg_interpolation::linearly_interpolate(first, second, -0.1, out), std::runtime_error);
  // t > second_timestamp
  EXPECT_THROW(acg_interpolation::linearly_interpolate(first, second, 2.0, out), std::runtime_error);
}

// ****** Definitions of the test parameters and test cases for task space point interpolation ****** //

// Define the parameters for the test
struct TaskSpacePointTestParameters
{
  acg_control_msgs::msg::TaskSpaceTrajectoryPoint first_point;
  acg_control_msgs::msg::TaskSpaceTrajectoryPoint second_point;
  double t;
  acg_control_msgs::msg::TaskSpacePoint expected_point;
};

class TaskSpacePointInterpolationTestSuite : public testing::TestWithParam<TaskSpacePointTestParameters>
{
public:
  void SetUp() override
  {
    const TaskSpacePointTestParameters params = GetParam();
    first_point = params.first_point;
    second_point = params.second_point;
    t = params.t;
    expected_point = params.expected_point;
  }

protected:
  acg_control_msgs::msg::TaskSpaceTrajectoryPoint first_point;
  acg_control_msgs::msg::TaskSpaceTrajectoryPoint second_point;
  double t;
  acg_control_msgs::msg::TaskSpacePoint expected_point;
  acg_control_msgs::msg::TaskSpacePoint interpolated_point;
};

/**
 * @brief Test the \c linearly_interpolate function of the \c interpolation library on task space points
 */
TEST_P(TaskSpacePointInterpolationTestSuite, SampleTaskSpacePoint)
{
  // Call the function under test
  acg_interpolation::linearly_interpolate(first_point, second_point, t, interpolated_point);

  // Validate the results
  EXPECT_EQ(interpolated_point, expected_point);
}

// Helper functions to create test points
TaskSpacePointTestParameters createTaskSpacePointNormalCase()
{
  TaskSpacePointTestParameters params;

  params.first_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
  params.first_point.point.pose.position.x = 0.0;
  params.first_point.point.pose.position.y = 0.0;
  params.first_point.point.pose.position.z = 0.0;
  params.first_point.point.pose.orientation.x = 0.0;
  params.first_point.point.pose.orientation.y = 0.0;
  params.first_point.point.pose.orientation.z = 0.0;
  params.first_point.point.pose.orientation.w = 1.0;
  params.first_point.point.twist.linear.x = 0.0;
  params.first_point.point.twist.linear.y = 0.0;
  params.first_point.point.twist.linear.z = 0.0;
  params.first_point.point.twist.angular.x = 0.0;
  params.first_point.point.twist.angular.y = 0.0;
  params.first_point.point.twist.angular.z = 0.0;
  params.first_point.point.acceleration.linear.x = 0.0;
  params.first_point.point.acceleration.linear.y = 0.0;
  params.first_point.point.acceleration.linear.z = 0.0;
  params.first_point.point.acceleration.angular.x = 0.0;
  params.first_point.point.acceleration.angular.y = 0.0;
  params.first_point.point.acceleration.angular.z = 0.0;
  params.first_point.point.wrench.force.x = 0.0;
  params.first_point.point.wrench.force.y = 0.0;
  params.first_point.point.wrench.force.z = 0.0;
  params.first_point.point.wrench.torque.x = 0.0;
  params.first_point.point.wrench.torque.y = 0.0;
  params.first_point.point.wrench.torque.z = 0.0;
  params.first_point.point.wrench_derivative.force.x = 0.0;
  params.first_point.point.wrench_derivative.force.y = 0.0;
  params.first_point.point.wrench_derivative.force.z = 0.0;
  params.first_point.point.wrench_derivative.torque.x = 0.0;
  params.first_point.point.wrench_derivative.torque.y = 0.0;
  params.first_point.point.wrench_derivative.torque.z = 0.0;

  params.second_point.time_from_start = rclcpp::Duration::from_seconds(1.0);
  params.second_point.point.pose.position.x = 1.0;
  params.second_point.point.pose.position.y = 1.0;
  params.second_point.point.pose.position.z = 1.0;
  params.second_point.point.pose.orientation.x = 0.0;
  params.second_point.point.pose.orientation.y = 0.0;
  params.second_point.point.pose.orientation.z = 0.0;
  params.second_point.point.pose.orientation.w = 1.0;
  params.second_point.point.twist.linear.x = 1.0;
  params.second_point.point.twist.linear.y = 1.0;
  params.second_point.point.twist.linear.z = 1.0;
  params.second_point.point.twist.angular.x = 1.0;
  params.second_point.point.twist.angular.y = 1.0;
  params.second_point.point.twist.angular.z = 1.0;
  params.second_point.point.acceleration.linear.x = 1.0;
  params.second_point.point.acceleration.linear.y = 1.0;
  params.second_point.point.acceleration.linear.z = 1.0;
  params.second_point.point.acceleration.angular.x = 1.0;
  params.second_point.point.acceleration.angular.y = 1.0;
  params.second_point.point.acceleration.angular.z = 1.0;
  params.second_point.point.wrench.force.x = 1.0;
  params.second_point.point.wrench.force.y = 1.0;
  params.second_point.point.wrench.force.z = 1.0;
  params.second_point.point.wrench.torque.x = 1.0;
  params.second_point.point.wrench.torque.y = 1.0;
  params.second_point.point.wrench.torque.z = 1.0;
  params.second_point.point.wrench_derivative.force.x = 1.0;
  params.second_point.point.wrench_derivative.force.y = 1.0;
  params.second_point.point.wrench_derivative.force.z = 1.0;
  params.second_point.point.wrench_derivative.torque.x = 1.0;
  params.second_point.point.wrench_derivative.torque.y = 1.0;
  params.second_point.point.wrench_derivative.torque.z = 1.0;

  params.t = 0.5;

  params.expected_point.pose.position.x = 0.5;
  params.expected_point.pose.position.y = 0.5;
  params.expected_point.pose.position.z = 0.5;
  params.expected_point.pose.orientation.x = 0.0;
  params.expected_point.pose.orientation.y = 0.0;
  params.expected_point.pose.orientation.z = 0.0;
  params.expected_point.pose.orientation.w = 1.0;
  params.expected_point.twist.linear.x = 0.5;
  params.expected_point.twist.linear.y = 0.5;
  params.expected_point.twist.linear.z = 0.5;
  params.expected_point.twist.angular.x = 0.5;
  params.expected_point.twist.angular.y = 0.5;
  params.expected_point.twist.angular.z = 0.5;
  params.expected_point.acceleration.linear.x = 0.5;
  params.expected_point.acceleration.linear.y = 0.5;
  params.expected_point.acceleration.linear.z = 0.5;
  params.expected_point.acceleration.angular.x = 0.5;
  params.expected_point.acceleration.angular.y = 0.5;
  params.expected_point.acceleration.angular.z = 0.5;
  params.expected_point.wrench.force.x = 0.5;
  params.expected_point.wrench.force.y = 0.5;
  params.expected_point.wrench.force.z = 0.5;
  params.expected_point.wrench.torque.x = 0.5;
  params.expected_point.wrench.torque.y = 0.5;
  params.expected_point.wrench.torque.z = 0.5;
  params.expected_point.wrench_derivative.force.x = 0.5;
  params.expected_point.wrench_derivative.force.y = 0.5;
  params.expected_point.wrench_derivative.force.z = 0.5;
  params.expected_point.wrench_derivative.torque.x = 0.5;
  params.expected_point.wrench_derivative.torque.y = 0.5;
  params.expected_point.wrench_derivative.torque.z = 0.5;
  return params;
}

TaskSpacePointTestParameters createTaskSpacePointEdgeCaseTimeZero()
{
  TaskSpacePointTestParameters params;

  params.first_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
  params.first_point.point.pose.position.x = 1.0;
  params.first_point.point.pose.position.y = 1.0;
  params.first_point.point.pose.position.z = 1.0;
  params.first_point.point.pose.orientation.x = 0.0;
  params.first_point.point.pose.orientation.y = 0.0;
  params.first_point.point.pose.orientation.z = 0.0;
  params.first_point.point.pose.orientation.w = 1.0;
  params.first_point.point.twist.linear.x = 1.0;
  params.first_point.point.twist.linear.y = 1.0;
  params.first_point.point.twist.linear.z = 1.0;
  params.first_point.point.twist.angular.x = 1.0;
  params.first_point.point.twist.angular.y = 1.0;
  params.first_point.point.twist.angular.z = 1.0;
  params.first_point.point.acceleration.linear.x = 1.0;
  params.first_point.point.acceleration.linear.y = 1.0;
  params.first_point.point.acceleration.linear.z = 1.0;
  params.first_point.point.acceleration.angular.x = 1.0;
  params.first_point.point.acceleration.angular.y = 1.0;
  params.first_point.point.acceleration.angular.z = 1.0;
  params.first_point.point.wrench.force.x = 1.0;
  params.first_point.point.wrench.force.y = 1.0;
  params.first_point.point.wrench.force.z = 1.0;
  params.first_point.point.wrench.torque.x = 1.0;
  params.first_point.point.wrench.torque.y = 1.0;
  params.first_point.point.wrench.torque.z = 1.0;
  params.first_point.point.wrench_derivative.force.x = 1.0;
  params.first_point.point.wrench_derivative.force.y = 1.0;
  params.first_point.point.wrench_derivative.force.z = 1.0;
  params.first_point.point.wrench_derivative.torque.x = 1.0;
  params.first_point.point.wrench_derivative.torque.y = 1.0;
  params.first_point.point.wrench_derivative.torque.z = 1.0;

  params.second_point.time_from_start = rclcpp::Duration::from_seconds(2.0);
  params.second_point.point.pose.position.x = 2.0;
  params.second_point.point.pose.position.y = 2.0;
  params.second_point.point.pose.position.z = 2.0;
  params.second_point.point.pose.orientation.x = 0.0;
  params.second_point.point.pose.orientation.y = 0.0;
  params.second_point.point.pose.orientation.z = 0.0;
  params.second_point.point.pose.orientation.w = 1.0;
  params.second_point.point.twist.linear.x = 2.0;
  params.second_point.point.twist.linear.y = 2.0;
  params.second_point.point.twist.linear.z = 2.0;
  params.second_point.point.twist.angular.x = 2.0;
  params.second_point.point.twist.angular.y = 2.0;
  params.second_point.point.twist.angular.z = 2.0;
  params.second_point.point.acceleration.linear.x = 2.0;
  params.second_point.point.acceleration.linear.y = 2.0;
  params.second_point.point.acceleration.linear.z = 2.0;
  params.second_point.point.acceleration.angular.x = 2.0;
  params.second_point.point.acceleration.angular.y = 2.0;
  params.second_point.point.acceleration.angular.z = 2.0;
  params.second_point.point.wrench.force.x = 2.0;
  params.second_point.point.wrench.force.y = 2.0;
  params.second_point.point.wrench.force.z = 2.0;
  params.second_point.point.wrench.torque.x = 2.0;
  params.second_point.point.wrench.torque.y = 2.0;
  params.second_point.point.wrench.torque.z = 2.0;
  params.second_point.point.wrench_derivative.force.x = 2.0;
  params.second_point.point.wrench_derivative.force.y = 2.0;
  params.second_point.point.wrench_derivative.force.z = 2.0;
  params.second_point.point.wrench_derivative.torque.x = 2.0;
  params.second_point.point.wrench_derivative.torque.y = 2.0;
  params.second_point.point.wrench_derivative.torque.z = 2.0;

  params.t = 0.0;

  params.expected_point.pose.position.x = 1.0;
  params.expected_point.pose.position.y = 1.0;
  params.expected_point.pose.position.z = 1.0;
  params.expected_point.pose.orientation.x = 0.0;
  params.expected_point.pose.orientation.y = 0.0;
  params.expected_point.pose.orientation.z = 0.0;
  params.expected_point.pose.orientation.w = 1.0;
  params.expected_point.twist.linear.x = 1.0;
  params.expected_point.twist.linear.y = 1.0;
  params.expected_point.twist.linear.z = 1.0;
  params.expected_point.twist.angular.x = 1.0;
  params.expected_point.twist.angular.y = 1.0;
  params.expected_point.twist.angular.z = 1.0;
  params.expected_point.acceleration.linear.x = 1.0;
  params.expected_point.acceleration.linear.y = 1.0;
  params.expected_point.acceleration.linear.z = 1.0;
  params.expected_point.acceleration.angular.x = 1.0;
  params.expected_point.acceleration.angular.y = 1.0;
  params.expected_point.acceleration.angular.z = 1.0;
  params.expected_point.wrench.force.x = 1.0;
  params.expected_point.wrench.force.y = 1.0;
  params.expected_point.wrench.force.z = 1.0;
  params.expected_point.wrench.torque.x = 1.0;
  params.expected_point.wrench.torque.y = 1.0;
  params.expected_point.wrench.torque.z = 1.0;
  params.expected_point.wrench_derivative.force.x = 1.0;
  params.expected_point.wrench_derivative.force.y = 1.0;
  params.expected_point.wrench_derivative.force.z = 1.0;
  params.expected_point.wrench_derivative.torque.x = 1.0;
  params.expected_point.wrench_derivative.torque.y = 1.0;
  params.expected_point.wrench_derivative.torque.z = 1.0;
  return params;
}

TaskSpacePointTestParameters createTaskSpacePointEdgeCaseTimeOne()
{
  TaskSpacePointTestParameters params;

  params.first_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
  params.first_point.point.pose.position.x = 1.0;
  params.first_point.point.pose.position.y = 1.0;
  params.first_point.point.pose.position.z = 1.0;
  params.first_point.point.pose.orientation.x = 0.0;
  params.first_point.point.pose.orientation.y = 0.0;
  params.first_point.point.pose.orientation.z = 0.0;
  params.first_point.point.pose.orientation.w = 1.0;
  params.first_point.point.twist.linear.x = 1.0;
  params.first_point.point.twist.linear.y = 1.0;
  params.first_point.point.twist.linear.z = 1.0;
  params.first_point.point.twist.angular.x = 1.0;
  params.first_point.point.twist.angular.y = 1.0;
  params.first_point.point.twist.angular.z = 1.0;
  params.first_point.point.acceleration.linear.x = 1.0;
  params.first_point.point.acceleration.linear.y = 1.0;
  params.first_point.point.acceleration.linear.z = 1.0;
  params.first_point.point.acceleration.angular.x = 1.0;
  params.first_point.point.acceleration.angular.y = 1.0;
  params.first_point.point.acceleration.angular.z = 1.0;
  params.first_point.point.wrench.force.x = 1.0;
  params.first_point.point.wrench.force.y = 1.0;
  params.first_point.point.wrench.force.z = 1.0;
  params.first_point.point.wrench.torque.x = 1.0;
  params.first_point.point.wrench.torque.y = 1.0;
  params.first_point.point.wrench.torque.z = 1.0;
  params.first_point.point.wrench_derivative.force.x = 1.0;
  params.first_point.point.wrench_derivative.force.y = 1.0;
  params.first_point.point.wrench_derivative.force.z = 1.0;
  params.first_point.point.wrench_derivative.torque.x = 1.0;
  params.first_point.point.wrench_derivative.torque.y = 1.0;
  params.first_point.point.wrench_derivative.torque.z = 1.0;

  params.second_point.time_from_start = rclcpp::Duration::from_seconds(2.0);
  params.second_point.point.pose.position.x = 2.0;
  params.second_point.point.pose.position.y = 2.0;
  params.second_point.point.pose.position.z = 2.0;
  params.second_point.point.pose.orientation.x = 0.0;
  params.second_point.point.pose.orientation.y = 0.0;
  params.second_point.point.pose.orientation.z = 0.0;
  params.second_point.point.pose.orientation.w = 1.0;
  params.second_point.point.twist.linear.x = 2.0;
  params.second_point.point.twist.linear.y = 2.0;
  params.second_point.point.twist.linear.z = 2.0;
  params.second_point.point.twist.angular.x = 2.0;
  params.second_point.point.twist.angular.y = 2.0;
  params.second_point.point.twist.angular.z = 2.0;
  params.second_point.point.acceleration.linear.x = 2.0;
  params.second_point.point.acceleration.linear.y = 2.0;
  params.second_point.point.acceleration.linear.z = 2.0;
  params.second_point.point.acceleration.angular.x = 2.0;
  params.second_point.point.acceleration.angular.y = 2.0;
  params.second_point.point.acceleration.angular.z = 2.0;
  params.second_point.point.wrench.force.x = 2.0;
  params.second_point.point.wrench.force.y = 2.0;
  params.second_point.point.wrench.force.z = 2.0;
  params.second_point.point.wrench.torque.x = 2.0;
  params.second_point.point.wrench.torque.y = 2.0;
  params.second_point.point.wrench.torque.z = 2.0;
  params.second_point.point.wrench_derivative.force.x = 2.0;
  params.second_point.point.wrench_derivative.force.y = 2.0;
  params.second_point.point.wrench_derivative.force.z = 2.0;
  params.second_point.point.wrench_derivative.torque.x = 2.0;
  params.second_point.point.wrench_derivative.torque.y = 2.0;
  params.second_point.point.wrench_derivative.torque.z = 2.0;

  params.t = 2.0;

  params.expected_point.pose.position.x = 2.0;
  params.expected_point.pose.position.y = 2.0;
  params.expected_point.pose.position.z = 2.0;
  params.expected_point.pose.orientation.x = 0.0;
  params.expected_point.pose.orientation.y = 0.0;
  params.expected_point.pose.orientation.z = 0.0;
  params.expected_point.pose.orientation.w = 1.0;
  params.expected_point.twist.linear.x = 2.0;
  params.expected_point.twist.linear.y = 2.0;
  params.expected_point.twist.linear.z = 2.0;
  params.expected_point.twist.angular.x = 2.0;
  params.expected_point.twist.angular.y = 2.0;
  params.expected_point.twist.angular.z = 2.0;
  params.expected_point.acceleration.linear.x = 2.0;
  params.expected_point.acceleration.linear.y = 2.0;
  params.expected_point.acceleration.linear.z = 2.0;
  params.expected_point.acceleration.angular.x = 2.0;
  params.expected_point.acceleration.angular.y = 2.0;
  params.expected_point.acceleration.angular.z = 2.0;
  params.expected_point.wrench.force.x = 2.0;
  params.expected_point.wrench.force.y = 2.0;
  params.expected_point.wrench.force.z = 2.0;
  params.expected_point.wrench.torque.x = 2.0;
  params.expected_point.wrench.torque.y = 2.0;
  params.expected_point.wrench.torque.z = 2.0;
  params.expected_point.wrench_derivative.force.x = 2.0;
  params.expected_point.wrench_derivative.force.y = 2.0;
  params.expected_point.wrench_derivative.force.z = 2.0;
  params.expected_point.wrench_derivative.torque.x = 2.0;
  params.expected_point.wrench_derivative.torque.y = 2.0;
  params.expected_point.wrench_derivative.torque.z = 2.0;
  return params;
}

// Instantiate test suite with test cases
INSTANTIATE_TEST_SUITE_P(TaskSpacePointInterpolationTests, TaskSpacePointInterpolationTestSuite,
                         testing::Values(createTaskSpacePointNormalCase(), createTaskSpacePointEdgeCaseTimeZero(),
                                         createTaskSpacePointEdgeCaseTimeOne()));

/**
 * @brief Test that the \c linearly_interpolate function of the \c interpolation library on task space points throws an exception when the point
 * timestamps are not in the correct order.
 */
TEST(TaskSpacePointInterpolationExceptionTest, ThrowsOnFirstTimestampGreaterOrEqual)
{
  acg_control_msgs::msg::TaskSpaceTrajectoryPoint first, second;
  first.time_from_start = rclcpp::Duration::from_seconds(2.0);
  second.time_from_start = rclcpp::Duration::from_seconds(1.0);
  acg_control_msgs::msg::TaskSpacePoint out;
  EXPECT_THROW(acg_interpolation::linearly_interpolate(first, second, 1.5, out), std::runtime_error);
}

/**
 * @brief Test that the \c linearly_interpolate function of the \c interpolation library on task space points throws an exception when the provided
 * time is out of bounds.
 */
TEST(TaskSpacePointInterpolationExceptionTest, ThrowsOnTimeOutOfBounds)
{
  acg_control_msgs::msg::TaskSpaceTrajectoryPoint first, second;
  first.time_from_start = rclcpp::Duration::from_seconds(0.0);
  second.time_from_start = rclcpp::Duration::from_seconds(1.0);
  acg_control_msgs::msg::TaskSpacePoint out;
  // t < first_timestamp
  EXPECT_THROW(acg_interpolation::linearly_interpolate(first, second, -0.1, out), std::runtime_error);
  // t > second_timestamp
  EXPECT_THROW(acg_interpolation::linearly_interpolate(first, second, 2.0, out), std::runtime_error);
}
