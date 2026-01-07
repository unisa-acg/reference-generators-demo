/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_force_torque_sensor_reader.cpp
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Apr 09, 2025
 *
 * This is a test file for the ForceTorqueSensorReader class, where a
 * black box testing approach is used.
 * All functions of the ForceTorqueSensorReader class are tested.
 *
 * -------------------------------------------------------------------
 */

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/wrench.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

#include "xxx_hardware_interface_facade/force_torque_sensor_reader.hpp"

namespace xxx_hardware_interface_facade
{

struct ForceTorqueSensorReaderParameters
{
  std::string sensor_name;
  geometry_msgs::msg::Wrench expected_state;
  std::vector<std::string> override_config;
};

/**
 * @brief Test for configured \c ForceTorqueSensorReader.
 */
class ForceTorqueSensorReaderTest : public testing::TestWithParam<ForceTorqueSensorReaderParameters>
{
public:
  void SetUp() override;

protected:
  geometry_msgs::msg::Wrench expected_state_;
  std::vector<std::string> expected_full_interface_names_;

  // Holds the simulated values for the force and torque components (force.x, force.y, force.z, torque.x, torque.y, torque.z)
  std::array<double, 6> values_;
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> state_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface> loaned_state_interfaces_;

  ForceTorqueSensorReader force_torque_sensor_reader_;
};

/**
 * @brief This method sets up the scenario given by the parameters of the test.
 */
void ForceTorqueSensorReaderTest::SetUp()
{
  const ForceTorqueSensorReaderParameters& params = WithParamInterface::GetParam();
  std::string sensor_name = params.sensor_name;
  expected_state_ = params.expected_state;
  std::vector<std::string> override_config = params.override_config;

  geometry_msgs::msg::Wrench sensor_state;

  values_[0] = expected_state_.force.x;
  values_[1] = expected_state_.force.y;
  values_[2] = expected_state_.force.z;
  values_[3] = expected_state_.torque.x;
  values_[4] = expected_state_.torque.y;
  values_[5] = expected_state_.torque.z;

  const std::array<std::string, 6> DEFAULT_INTERFACE_NAMES = { "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z" };

  expected_full_interface_names_.resize(6);
  state_interfaces_.resize(6);
  if (override_config.empty())
  {
    for (std::size_t i = 0; i < 6; i++)
    {
      expected_full_interface_names_[i] = sensor_name + "/" + DEFAULT_INTERFACE_NAMES[i];
      state_interfaces_[i] = std::make_shared<hardware_interface::StateInterface>(sensor_name, DEFAULT_INTERFACE_NAMES[i], &values_[i]);
    }
  }
  else
  {
    expected_full_interface_names_ = override_config;

    for (std::size_t i = 0; i < 6; i++)
    {
      std::size_t pos = expected_full_interface_names_[i].find_last_of("/");
      std::string prefix = expected_full_interface_names_[i].substr(0, pos);
      std::string interface_name = expected_full_interface_names_[i].substr(pos + 1);
      state_interfaces_[i] = std::make_shared<hardware_interface::StateInterface>(prefix, interface_name, &values_[i]);
    }
  }
  // Configure the state interfaces
  ASSERT_TRUE(force_torque_sensor_reader_.configure_state_interfaces(sensor_name, override_config));

  for (std::size_t i = 0; i < 6; i++)
  {
    loaned_state_interfaces_.push_back(hardware_interface::LoanedStateInterface(*state_interfaces_[i]));
  }
  ASSERT_TRUE(force_torque_sensor_reader_.assign_loaned_state_interfaces(loaned_state_interfaces_));
}

/**
 * @brief Returns the test parameters for testing a force/torque sensor reader with default names.
 *
 * @return A ForceTorqueSensorReaderParameters object.
 */
ForceTorqueSensorReaderParameters getForceTorqueSensorReaderDefaultTestParams()
{
  ForceTorqueSensorReaderParameters params;
  params.sensor_name = "ft_sensor";
  params.expected_state.force.x = 1.0;
  params.expected_state.force.y = 2.0;
  params.expected_state.force.z = 3.0;
  params.expected_state.torque.x = 0.11;
  params.expected_state.torque.y = 0.22;
  params.expected_state.torque.z = 0.33;
  params.override_config = {};
  return params;
}

/**
 * @brief Returns the test parameters for testing a force/torque sensor reader with overridden names.
 *
 * @return A ForceTorqueSensorReaderParameters object.
 */
ForceTorqueSensorReaderParameters getForceTorqueSensorReaderOverrideTestParams()
{
  ForceTorqueSensorReaderParameters params;
  params.sensor_name = "ignition_ft_sensor";
  params.expected_state.force.x = 10.0;
  params.expected_state.force.y = 20.0;
  params.expected_state.force.z = 30.0;
  params.expected_state.torque.x = 1.1;
  params.expected_state.torque.y = 2.2;
  params.expected_state.torque.z = 3.3;
  params.override_config = { "ignition_ft_sensor/wrench.1", "ignition_ft_sensor/wrench.2", "ignition_ft_sensor/wrench.3",
                             "ignition_ft_sensor/wrench.4", "ignition_ft_sensor/wrench.5", "ignition_ft_sensor/wrench.6" };
  return params;
}

/**
 * @brief This test checks the available state interfaces names.
 */
TEST_P(ForceTorqueSensorReaderTest, TestOverrideInterfaceNamesAvailableStateInterfaces)
{
  std::vector<std::string> state_interfaces_names = force_torque_sensor_reader_.available_state_interfaces();

  // Checking the values returned by the available_state_interfaces method with the expected ones
  ASSERT_EQ(state_interfaces_names.size(), expected_full_interface_names_.size());
  for (std::size_t i = 0; i < state_interfaces_names.size(); i++)
  {
    EXPECT_EQ(state_interfaces_names[i], expected_full_interface_names_[i]);
  }
}

/**
 * @brief This test checks if the state interfaces are correctly read when using custom names.
 */
TEST_P(ForceTorqueSensorReaderTest, TestOverrideInterfaceNamesReadStateInterfaces)
{
  geometry_msgs::msg::Wrench sensor_state;
  force_torque_sensor_reader_.read_state_interfaces(sensor_state);

  EXPECT_EQ(sensor_state.force, expected_state_.force);
  EXPECT_EQ(sensor_state.torque, expected_state_.torque);
}

INSTANTIATE_TEST_SUITE_P(ForceTorqueSensorReaderTests, ForceTorqueSensorReaderTest,
                         testing::Values(getForceTorqueSensorReaderDefaultTestParams(), getForceTorqueSensorReaderOverrideTestParams()));

// *********** TESTS FOR UNCONFIGURED ForceTorqueSensorReader *********** //

/**
 * @brief This test tries to retrieve the available state interfaces from an unconfigured force/torque sensor reader.
 */
TEST(UnconfiguredForceTorqueSensorReaderTest, TestAvailableStateInterfacesWhenNotConfigured)
{
  ForceTorqueSensorReader force_torque_sensor_reader;
  EXPECT_TRUE(force_torque_sensor_reader.available_state_interfaces().empty());
}

/**
 * @brief This test tries to read the state interfaces from an unconfigured force/torque sensor reader.
 */
TEST(UnconfiguredForceTorqueSensorReaderTest, TestReadStateInterfacesWhenNotConfigured)
{
  // Creating a state message to read from the state interfaces
  geometry_msgs::msg::Wrench state;
  state.force.x = std::numeric_limits<double>::quiet_NaN();
  state.force.y = std::numeric_limits<double>::quiet_NaN();
  state.force.z = std::numeric_limits<double>::quiet_NaN();
  state.torque.x = std::numeric_limits<double>::quiet_NaN();
  state.torque.y = std::numeric_limits<double>::quiet_NaN();
  state.torque.z = std::numeric_limits<double>::quiet_NaN();

  ForceTorqueSensorReader force_torque_sensor_reader;
  force_torque_sensor_reader.read_state_interfaces(state);

  // Verify that the state has not been modified
  EXPECT_TRUE(std::isnan(state.force.x));
  EXPECT_TRUE(std::isnan(state.force.y));
  EXPECT_TRUE(std::isnan(state.force.z));
  EXPECT_TRUE(std::isnan(state.torque.x));
  EXPECT_TRUE(std::isnan(state.torque.y));
  EXPECT_TRUE(std::isnan(state.torque.z));
}

}  // namespace xxx_hardware_interface_facade
