/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_math_utilities.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Sep 22, 2025
 *
 * This is a test for the math utilities library.
 *
 * -------------------------------------------------------------------
 */

// Gtest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Dense>

// File under tests
#include "xxx_common_libraries/math_utilities.hpp"

struct VectorTestParameters
{
  Eigen::VectorXd current;
  Eigen::VectorXd last;
  std::vector<double> torque_rate_limits;
  Eigen::VectorXd expected_vector;
  bool expected_flag;
};

class VectorSaturateRateTestSuite : public testing::TestWithParam<VectorTestParameters>
{
public:
  void SetUp() override
  {
    const VectorTestParameters params = GetParam();
    current = params.current;
    last = params.last;
    torque_rate_limits = params.torque_rate_limits;
    expected_vector = params.expected_vector;
    expected_flag = params.expected_flag;
  }

protected:
  Eigen::VectorXd current;
  Eigen::VectorXd last;
  std::vector<double> torque_rate_limits;
  Eigen::VectorXd expected_vector;
  bool expected_flag;
};

/**
 * @brief Test for vector rate saturation functionality
 */
TEST_P(VectorSaturateRateTestSuite, SaturateRate)
{
  bool result = xxx_math_utilities::saturate_rate(last, current, torque_rate_limits);

  // Use Eigen's isApprox for floating point comparison
  EXPECT_TRUE(current.isApprox(expected_vector, 1e-9)) << "Expected: " << expected_vector.transpose() << ", Got: " << current.transpose();
  EXPECT_EQ(result, expected_flag);
}

// Helper functions to create test parameters
VectorTestParameters createVectorNormalCase()
{
  VectorTestParameters params;
  params.current = Eigen::Vector3d(1.0, 2.0, 3.0);
  params.last = Eigen::Vector3d(0.0, 0.0, 0.0);
  params.torque_rate_limits = std::vector<double>{ 1.5, 1.5, 1.5 };
  params.expected_vector = Eigen::Vector3d(1.0, 1.5, 1.5);
  params.expected_flag = true;
  return params;
}

VectorTestParameters createVectorEqualCase()
{
  VectorTestParameters params;
  params.current = Eigen::Vector3d(1.0, 1.0, 1.0);
  params.last = Eigen::Vector3d(1.0, 1.0, 1.0);
  params.torque_rate_limits = std::vector<double>{ 1.5, 1.5, 1.5 };
  params.expected_vector = Eigen::Vector3d(1.0, 1.0, 1.0);
  params.expected_flag = true;
  return params;
}

VectorTestParameters createVectorSkipJoint()
{
  VectorTestParameters params;
  params.current = Eigen::Vector3d(1.0, 2.0, 1.0);
  params.last = Eigen::Vector3d(0.0, 0.0, 0.0);
  params.torque_rate_limits = std::vector<double>{ 1.5, 0.0, 1.5 };
  params.expected_vector = Eigen::Vector3d(1.0, 2.0, 1.0);
  params.expected_flag = true;
  return params;
}

VectorTestParameters createVectorEdgeCaseEmptyLimits()
{
  VectorTestParameters params;
  params.current = Eigen::Vector3d(1.0, 2.0, 3.0);
  params.last = Eigen::Vector3d(0.0, 0.0, 0.0);
  params.torque_rate_limits = std::vector<double>{};
  params.expected_vector = Eigen::Vector3d(1.0, 2.0, 3.0);
  params.expected_flag = false;
  return params;
}

VectorTestParameters createVectorEdgeCaseVectorSizeMismatch1()
{
  VectorTestParameters params;
  params.current = Eigen::Vector2d(1.0, 2.0);
  params.last = Eigen::Vector3d(0.0, 0.0, 0.0);
  params.torque_rate_limits = std::vector<double>{ 1.5, 1.5, 1.5 };
  params.expected_vector = Eigen::Vector2d(1.0, 2.0);
  params.expected_flag = false;
  return params;
}

VectorTestParameters createVectorEdgeCaseVectorSizeMismatch2()
{
  VectorTestParameters params;
  params.current = Eigen::Vector3d(1.0, 2.0, 3.0);
  params.last = Eigen::Vector3d(0.0, 0.0, 0.0);
  params.torque_rate_limits = std::vector<double>{ 1.5, 1.5 };
  params.expected_vector = Eigen::Vector3d(1.0, 2.0, 3.0);
  params.expected_flag = false;
  return params;
}

VectorTestParameters createVectorEdgeCaseNegativeRate()
{
  VectorTestParameters params;
  params.current = Eigen::Vector3d(1.0, 2.0, 3.0);
  params.last = Eigen::Vector3d(0.0, 0.0, 0.0);
  params.torque_rate_limits = std::vector<double>{ 1.5, -1.5, 1.5 };
  params.expected_vector = Eigen::Vector3d(1.0, 2.0, 1.5);
  params.expected_flag = false;
  return params;
}

INSTANTIATE_TEST_SUITE_P(VectorSaturateRateTests, VectorSaturateRateTestSuite,
                         testing::Values(createVectorNormalCase(), createVectorEqualCase(), createVectorSkipJoint(),
                                         createVectorEdgeCaseEmptyLimits(), createVectorEdgeCaseVectorSizeMismatch1(),
                                         createVectorEdgeCaseVectorSizeMismatch2(), createVectorEdgeCaseNegativeRate()));
