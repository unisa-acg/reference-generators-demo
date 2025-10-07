/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_state_reader.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Mar 3, 2025
 *
 * This is a test file for the StateReader class, where a black box
 * testing approach is used.
 * All functions of the StateReader class are tested.
 *
 * -------------------------------------------------------------------
 */

#include <string>
#include <vector>
#include <map>
#include <memory>

#include <gtest/gtest.h>

#include <hardware_interface/loaned_state_interface.hpp>

#include "acg_hardware_interface_facade/state_reader.hpp"

namespace acg_hardware_interface_facade
{

struct StateReaderParameters
{
  std::vector<std::string> state_interfaces_names;
  std::vector<std::string> joint_names;
  std::string robot_name;
  std::map<std::string, std::vector<double>> exp_robot_state_map_;
  StateInterfaceNamesOverrideConfig override_config;
};

class StateReaderTest : public testing::TestWithParam<StateReaderParameters>
{
public:
  void SetUp() override;

protected:
  // common variables used by the tests
  std::vector<std::string> state_interfaces_names_;
  std::vector<std::string> joint_names_;
  std::string robot_name_;
  std::size_t num_joints_{ 0 };
  std::size_t total_interfaces_size_{ 0 };

  std::map<std::string, std::vector<double>> exp_robot_state_map_;
  std::vector<std::string> expected_full_interface_names_;

  // The vector of doubles bound to the state interfaces
  std::vector<double> values_;
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> state_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface> loaned_state_interfaces_;

  acg_hardware_interface_facade::RobotJointState state_;

  StateReader state_reader_;
};

/**
 * @brief This method sets up the scenario given by the parameters of the test.
 */
void StateReaderTest::SetUp()
{
  const StateReaderParameters srp = WithParamInterface::GetParam();
  state_interfaces_names_ = srp.state_interfaces_names;
  joint_names_ = srp.joint_names;
  robot_name_ = srp.robot_name;
  exp_robot_state_map_ = srp.exp_robot_state_map_;
  StateInterfaceNamesOverrideConfig override_config = srp.override_config;

  num_joints_ = joint_names_.size();
  total_interfaces_size_ = num_joints_ * state_interfaces_names_.size();

  state_.positions.assign(num_joints_, 0.0);
  state_.velocities.assign(num_joints_, 0.0);
  state_.accelerations.assign(num_joints_, 0.0);
  state_.efforts.assign(num_joints_, 0.0);

  values_.resize(total_interfaces_size_);
  std::fill(values_.begin(), values_.end(), 0.0);

  const std::map<std::string, std::vector<std::string>> override_config_map = { { "position", override_config.position_state_interfaces },
                                                                                { "velocity", override_config.velocity_state_interfaces },
                                                                                { "acceleration", override_config.acceleration_state_interfaces },
                                                                                { "effort", override_config.effort_state_interfaces } };

  state_interfaces_.resize(total_interfaces_size_);
  std::size_t i = 0;
  std::string full_interface_name;
  for (const std::string& interface : state_interfaces_names_)
  {
    for (const std::string& joint : joint_names_)
    {
      std::vector<std::string> override_config_names = override_config_map.at(interface);
      bool is_override_config_empty = override_config_names.empty();
      if (is_override_config_empty)
      {
        full_interface_name = robot_name_ + "/" + joint + "/" + interface;
      }
      else
      {
        full_interface_name = override_config_names[i % num_joints_];
      }
      full_interface_name = is_override_config_empty ? robot_name_ + "/" + joint + "/" + interface : override_config_names[i % num_joints_];
      expected_full_interface_names_.push_back(full_interface_name);
      values_[i] = exp_robot_state_map_[interface][i % num_joints_];  // i % num_joints_ to cycle the values of the exp_robot_state_map_
      if (is_override_config_empty)
      {
        state_interfaces_[i] = std::make_shared<hardware_interface::StateInterface>(robot_name_ + "/" + joint, interface, &values_[i]);
      }
      else
      {
        // strip the prefix from the full_interface_name
        std::size_t pos = full_interface_name.find_last_of("/");
        std::string prefix = full_interface_name.substr(0, pos);
        std::string interface_name = full_interface_name.substr(pos + 1);
        state_interfaces_[i] = std::make_shared<hardware_interface::StateInterface>(prefix, interface_name, &values_[i]);
      }

      i++;
    }
  }

  // Configure the state interfaces
  ASSERT_TRUE(state_reader_.configure_state_interfaces(state_interfaces_names_, joint_names_, robot_name_, override_config));

  for (const std::shared_ptr<hardware_interface::StateInterface>& state_interface : state_interfaces_)
  {
    loaned_state_interfaces_.push_back(hardware_interface::LoanedStateInterface(*state_interface));
  }

  EXPECT_TRUE(state_reader_.assign_loaned_state_interfaces(loaned_state_interfaces_));
}

/**
 * @brief This test checks if all state interfaces are available.
 */
TEST_P(StateReaderTest, TestStateInterfacesAvailability)
{
  EXPECT_TRUE(state_reader_.has_joint_position_state_interface());
  EXPECT_TRUE(state_reader_.has_joint_velocity_state_interface());
  EXPECT_TRUE(state_reader_.has_joint_acceleration_state_interface());
  EXPECT_TRUE(state_reader_.has_joint_effort_state_interface());
}

/**
 * @brief Returns the test parameters for testing a state reader with default names.
 *
 * @return A StateReaderParameters object.
 */
StateReaderParameters getStateReaderDefaultTestParams()
{
  StateReaderParameters params;
  params.robot_name = "robot";
  params.state_interfaces_names = { "position", "velocity", "acceleration", "effort" };
  params.joint_names = { "joint1", "joint2", "joint3" };
  params.exp_robot_state_map_ = { { "position", { 1.0, 2.0, 3.0 } },
                                  { "velocity", { 0.11, 0.22, 0.33 } },
                                  { "acceleration", { 0.111, 0.222, 0.333 } },
                                  { "effort", { 0.1111, 0.2222, 0.3333 } } };
  return params;
}

/**
 * @brief Returns the test parameters for testing a state reader with overridden names.
 *
 * @return A StateReaderParameters object.
 */
StateReaderParameters getStateReaderOverrideTestParams()
{
  StateReaderParameters params;
  params.robot_name = "robot";
  params.state_interfaces_names = { "position", "velocity", "acceleration", "effort" };
  params.joint_names = { "joint1", "joint2", "joint3" };
  params.exp_robot_state_map_ = { { "position", { 4.0, 5.0, 6.0 } },
                                  { "velocity", { 0.44, 0.55, 0.66 } },
                                  { "acceleration", { 0.444, 0.555, 0.666 } },
                                  { "effort", { 0.4444, 0.5555, 0.6666 } } };
  params.override_config.position_state_interfaces = { "robot_joint1/position", "robot_joint2/position", "robot_joint3/position" };
  params.override_config.velocity_state_interfaces = { "robot_joint1/velocity", "robot_joint2/velocity", "robot_joint3/velocity" };

  return params;
}

/**
 * @brief This test checks the available state interfaces names.
 */
TEST_P(StateReaderTest, TestOverrideInterfaceNamesAvailableStateInterfaces)
{
  std::vector<std::string> state_interfaces_names = state_reader_.available_state_interfaces();

  // Checking the values returned by the available_state_interfaces method with the expected ones
  ASSERT_EQ(state_interfaces_names.size(), total_interfaces_size_);
  for (std::size_t i = 0; i < state_interfaces_names.size(); i++)
  {
    EXPECT_EQ(state_interfaces_names[i], expected_full_interface_names_[i]);
  }
}

/**
 * @brief This test checks if the state interfaces are correctly read when using custom names.
 */
TEST_P(StateReaderTest, TestOverrideInterfaceNamesReadStateInterfaces)
{
  state_reader_.read_state_interfaces(state_);

  // Check the values if they are correctly read
  for (std::size_t i = 0; i < joint_names_.size(); i++)
  {
    EXPECT_EQ(state_.positions[i], exp_robot_state_map_["position"][i]);
    EXPECT_EQ(state_.velocities[i], exp_robot_state_map_["velocity"][i]);
    EXPECT_EQ(state_.accelerations[i], exp_robot_state_map_["acceleration"][i]);
    EXPECT_EQ(state_.efforts[i], exp_robot_state_map_["effort"][i]);
  }
}

INSTANTIATE_TEST_SUITE_P(StateReaderTests, StateReaderTest, testing::Values(getStateReaderDefaultTestParams(), getStateReaderOverrideTestParams()));

// *********** TESTS FOR UNCONFIGURED STATE READER *********** //

class UnconfiguredStateReaderTest : public testing::Test
{
protected:
  StateReader state_reader_;
};

/**
 * @brief This test tries to retrieve the available state interfaces from an unconfigured state reader.
 */
TEST_F(UnconfiguredStateReaderTest, TestAvailableStateInterfacesWhenNotConfigured)
{
  EXPECT_TRUE(state_reader_.available_state_interfaces().empty());
}

/**
 * @brief This test tries to read the state interfaces from an unconfigured state reader.
 */
TEST_F(UnconfiguredStateReaderTest, TestReadStateInterfacesWhenNotConfigured)
{
  // Creating a state message to read from the command interfaces
  acg_hardware_interface_facade::RobotJointState state;
  const std::size_t num_joints{ 3 };

  state.positions.assign(num_joints, std::numeric_limits<double>::quiet_NaN());
  state.velocities.assign(num_joints, std::numeric_limits<double>::quiet_NaN());
  state.accelerations.assign(num_joints, std::numeric_limits<double>::quiet_NaN());
  state.efforts.assign(num_joints, std::numeric_limits<double>::quiet_NaN());

  state_reader_.read_state_interfaces(state);

  // checking that the state hasn't been modified
  for (std::size_t i = 0; i < num_joints; i++)
  {
    EXPECT_TRUE(std::isnan(state.positions[i]));
    EXPECT_TRUE(std::isnan(state.velocities[i]));
    EXPECT_TRUE(std::isnan(state.accelerations[i]));
    EXPECT_TRUE(std::isnan(state.efforts[i]));
  }
}

}  // namespace acg_hardware_interface_facade
