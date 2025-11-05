/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_unbiasing_filter.cpp
 * Author:  Alessio Coone
 * Org.:    UNISA
 * Date:    Sep 30, 2024
 *
 * This class provides a set of tests for the UnbiasingFilter class.
 *
 * -------------------------------------------------------------------
 */

#include <gtest/gtest.h>
#include "unbiasing_filter/unbiasing_filter.hpp"

// This class shares parameters and data across all tests
class SharedData
{
  friend class UnbiasingFilterTest;

  std::shared_ptr<rclcpp::Node> node_;

  SharedData(const SharedData&) = delete;  // this is a singleton
  SharedData()
  {
    initialize();
  }

  void initialize()
  {
    // Instantiate the node
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // Force rename of the node. Without this instruction,
    // the test case would rename all nodes to the same name,
    // causing conflicts.
    node_options.arguments({ "--ros-args", "-r", "__node:=unbiasing_filter_test" });
    node_ = rclcpp::Node::make_shared("unbiasing_filter_test", node_options);

    // Declare node's parameter
    node_->declare_parameter("number_of_stable_samples_for_bias", 0);
    node_->declare_parameter("tolerance", 0.0);

    // Set parameters
    node_->set_parameter(rclcpp::Parameter("number_of_stable_samples_for_bias", 10));
    node_->set_parameter(rclcpp::Parameter("tolerance", 0.0000001));
  }

public:
  static const SharedData& instance()
  {
    static SharedData instance;
    return instance;
  }
};

class UnbiasingFilterTest : public ::testing::Test
{
protected:
  void operator=(const SharedData& data)
  {
    node = data.node_;
  }

  void SetUp() override
  {
    *this = SharedData::instance();

    filter = std::make_shared<unbiasing_filter::UnbiasingFilter<double>>();
  }

public:
  /**
   * @brief Number of channels used in the multi channel filter.
   */
  static const uint8_t NUMBER_OF_CHANNELS = 7;

  /**
   * @brief Maximum number of reads to reach the stable state.
   */
  static const uint8_t MAX_READS = 20;

  /**
   * @brief Pointer to the instance of the derived filter class.
   */
  std::shared_ptr<unbiasing_filter::UnbiasingFilter<double>> filter;

  /**
   * @brief Shared pointer to the ROS2 node used for parameter handling.
   */
  rclcpp::Node::SharedPtr node;
};

/**
 * @brief Asserts the filter configuration is successful.
 */
TEST_F(UnbiasingFilterTest, testFilterConfiguration)
{
  ASSERT_TRUE(filter->configure(NUMBER_OF_CHANNELS,
                                "",                                    // Prefix for parameters
                                "UnbiasingFilter",                     // Filter name
                                node->get_node_logging_interface(),    // Node logging interface
                                node->get_node_parameters_interface()  // Node parameters interface
                                ));
}

/**
 * @brief Asserts that the filter at the beginning has not computed the bias.
 */
TEST_F(UnbiasingFilterTest, testFilterConfigurationState)
{
  ASSERT_TRUE(filter->configure(NUMBER_OF_CHANNELS,
                                "",                                    // Prefix for parameters
                                "UnbiasingFilter",                     // Filter name
                                node->get_node_logging_interface(),    // Node logging interface
                                node->get_node_parameters_interface()  // Node parameters interface
                                ));

  std::vector<double> data_in = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0 };
  std::vector<double> data_out(NUMBER_OF_CHANNELS, 0);

  filter->update(data_in, data_out);

  ASSERT_EQ(data_out[NUMBER_OF_CHANNELS - 1], 0.0);
}

/**
 * @brief This test verifies the filter does not compute the bias if
 * unstable samples are input.
 */
TEST_F(UnbiasingFilterTest, testFilterBiasComputingFail)
{
  ASSERT_TRUE(filter->configure(NUMBER_OF_CHANNELS,
                                "",                                    // Prefix for parameters
                                "UnbiasingFilter",                     // Filter name
                                node->get_node_logging_interface(),    // Node logging interface
                                node->get_node_parameters_interface()  // Node parameters interface
                                ));

  std::vector<double> data_in_low = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0 };
  std::vector<double> data_in_high = { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 0.0 };
  std::vector<double> data_out(NUMBER_OF_CHANNELS, 0.0);

  for (int i = 0; i < MAX_READS; i++)
  {
    if (i % 2 == 0)
    {
      filter->update(data_in_low, data_out);
    }
    else
    {
      filter->update(data_in_high, data_out);
    }
    ASSERT_EQ(data_out[NUMBER_OF_CHANNELS - 1], 0.0);
  }
}

/**
 * @brief Test the correct operation of the filter.
 */
TEST_F(UnbiasingFilterTest, testFilterOperation)
{
  ASSERT_TRUE(filter->configure(NUMBER_OF_CHANNELS,
                                "",                                    // Prefix for parameters
                                "UnbiasingFilter",                     // Filter name
                                node->get_node_logging_interface(),    // Node logging interface
                                node->get_node_parameters_interface()  // Node parameters interface
                                ));

  std::vector<double> expected_value = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  std::vector<double> data_in = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0 };
  std::vector<double> data_out(NUMBER_OF_CHANNELS, 0.0);

  auto number_of_samples_for_bias_ = MAX_READS / 2;

  for (int i = 0; i < number_of_samples_for_bias_ + 1; i++)
  {
    filter->update(data_in, data_out);

    if (i < number_of_samples_for_bias_)
    {
      ASSERT_EQ(data_out[NUMBER_OF_CHANNELS - 1], 0.0);
    }
  }

  for (int j = 0; j < NUMBER_OF_CHANNELS; j++)
  {
    ASSERT_EQ(data_out[j], expected_value[j]);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
