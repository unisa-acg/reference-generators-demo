/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_moving_average_poses_filter.cpp
 * Author:  Michele Marsico, Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Oct 24, 2024
 *
 * This class provides a set of tests for the
 * MultiChannelMovingAverageFilter class.
 *
 * -------------------------------------------------------------------
 */

#include <gtest/gtest.h>

#include <filters/mean.hpp>

#include "moving_average_poses_filter/moving_average_poses_filter.hpp"

using namespace std::chrono_literals;

// This class shares parameters and data across all tests
class SharedData
{
  friend class MovingAveragePosesFilterTest;

  // Private members
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
    node_options.arguments({ "--ros-args", "-r", "__node:=moving_average_poses_filter_test" });
    node_ = rclcpp::Node::make_shared("moving_average_poses_filter_test", node_options);

    // Declare node's parameter
    node_->declare_parameter("number_of_observations", 1);
  }

public:
  static const SharedData& instance()
  {
    static SharedData instance;
    return instance;
  }
};

class MovingAveragePosesFilterTest : public ::testing::Test
{
protected:
  void operator=(const SharedData& data)
  {
    node = data.node_;
  }

  void SetUp() override
  {
    *this = SharedData::instance();

    filter = std::make_shared<moving_average_poses_filter::MovingAveragePosesFilter<double>>();
  }

public:
  /**
   * @brief Shared pointer to the ROS2 node used for parameter handling.
   */
  rclcpp::Node::SharedPtr node;

  /**
   * @brief Pointer to the instance of the derived filter class.
   */
  std::shared_ptr<moving_average_poses_filter::MovingAveragePosesFilter<double>> filter;

  /**
   * @brief Maximum allowed difference to consider two numbers as equal.
   */
  const double EPSILON = 1e-4;

  /**
   * @brief Number of channels used in the moving average poses filter.
   */
  const uint8_t NUMBER_OF_CHANNELS = 7;
};

/**
 * @brief Asserts the filter configuration is successful.
 */
TEST_F(MovingAveragePosesFilterTest, testFilterConfiguration)
{
  node->set_parameter(rclcpp::Parameter("number_of_observations", 2));

  ASSERT_TRUE(filter->configure(NUMBER_OF_CHANNELS,
                                "",                                    // Prefix for parameters
                                "TwoSamplesWindow",                    // Filter name
                                node->get_node_logging_interface(),    // Node logging interface
                                node->get_node_parameters_interface()  // Node parameters interface
                                ));
}

/**
 * @brief Toggles the 7-channels input between { 1.0, 2.0, 3.0, 0.0, 0.0, 0.70711, 0.70711 } and
 * { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 }.
 * Checks that the mean is computed correctly over a 2-sized observation window, over 1e3 samples.
 */
TEST_F(MovingAveragePosesFilterTest, testTwoSamplesWindowFilterDynamic)
{
  node->set_parameter(rclcpp::Parameter("number_of_observations", 2));

  ASSERT_TRUE(filter->configure(NUMBER_OF_CHANNELS,
                                "",                                                        // Prefix for parameters
                                "TwoSamplesWindowDynamicMultiChannelMovingAverageFilter",  // Filter name
                                node->get_node_logging_interface(),                        // Node logging interface
                                node->get_node_parameters_interface()                      // Node parameters interface
                                ));

  const std::vector<double> DATA_IN_HIGH = { 1.0, 2.0, 3.0, 0.0, 0.0, 0.70711, 0.70711 };
  const std::vector<double> DATA_IN_LOW = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  // Since we are using a 2-sized window, the expected final value is the initial divided by two.
  const std::vector<double> FINAL_EXPECTED_VALUE = { 0.5, 1, 1.5, 0.0, 0.0, 0.38268, 0.92388 };

  std::vector<double> expected_value;

  std::vector<double> data_in = DATA_IN_HIGH;
  std::vector<double> data_out(NUMBER_OF_CHANNELS, 0);

  // Total number of samples to be processed
  const unsigned short int TOTAL_SAMPLES = 1e3;

  for (std::size_t i = 0; i < TOTAL_SAMPLES; i++)
  {
    // Update next input
    if (i % 2 == 1)
      data_in = DATA_IN_LOW;
    else
      data_in = DATA_IN_HIGH;

    // Update expected mean value
    expected_value = (i == 0) ? DATA_IN_HIGH : FINAL_EXPECTED_VALUE;

    // Call filter update
    filter->update(data_in, data_out);

    // Assert values are correct for each channel
    for (std::size_t j = 0; j < NUMBER_OF_CHANNELS; j++)
      ASSERT_NEAR(expected_value[j], data_out[j], EPSILON);
  }
}

/**
 * @brief Test the mean computation over a 5000-sized observation window, in an array with 1e5 samples.
 * The observed value is static.
 */
TEST_F(MovingAveragePosesFilterTest, testFiveThousandSamplesWindowFilterStatic)
{
  node->set_parameter(rclcpp::Parameter("number_of_observations", 5000));

  ASSERT_TRUE(filter->configure(NUMBER_OF_CHANNELS,
                                "",                                                   // Prefix for parameters
                                "FiveThousandSamplesWindowMovingAveragePosesFilter",  // Filter name
                                node->get_node_logging_interface(),                   // Node logging interface
                                node->get_node_parameters_interface()                 // Node parameters interface
                                ));

  // Total number of samples to be processed
  const unsigned int TOTAL_SAMPLES = 1e5;

  // Input vector
  const std::vector<double> DATA_IN = { 1.0, 2.0, 3.0, 0.0, 0.0, 0.70711, 0.70711 };

  // The filter expects a data_out vector of same size as the input.
  std::vector<double> data_out(NUMBER_OF_CHANNELS, 0);

  for (std::size_t i = 0; i < TOTAL_SAMPLES; i++)
  {
    filter->update(DATA_IN, data_out);

    // Assert values are correct for each channel
    for (std::size_t j = 0; j < NUMBER_OF_CHANNELS; j++)
      ASSERT_NEAR(DATA_IN[j], data_out[j], EPSILON);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
