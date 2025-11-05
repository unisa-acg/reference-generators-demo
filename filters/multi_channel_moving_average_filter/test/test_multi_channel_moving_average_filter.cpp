/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_multi_channel_moving_average_filter.cpp
 * Author:  Alessio Coone
 * Org.:    UNISA
 * Date:    Sep 30, 2024
 *
 * This class provides a set of tests for the
 * MultiChannelMovingAverageFilter class.
 *
 * -------------------------------------------------------------------
 */

#include <gtest/gtest.h>
#include <filters/mean.hpp>
#include "multi_channel_moving_average_filter/multi_channel_moving_average_filter.hpp"

using namespace std::chrono_literals;

// This class shares parameters and data across all tests
class SharedData
{
  friend class MultiChannelMovingAverageFilterTest;

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
    node_options.arguments({ "--ros-args", "-r", "__node:=multi_channel_moving_average_filter_test" });
    node_ = rclcpp::Node::make_shared("multi_channel_moving_average_filter_test", node_options);

    // Declare node's parameter
    node_->declare_parameter("number_of_observations", 0);
  }

public:
  static const SharedData& instance()
  {
    static SharedData instance;
    return instance;
  }
};

class MultiChannelMovingAverageFilterTest : public ::testing::Test
{
protected:
  void operator=(const SharedData& data)
  {
    node = data.node_;
  }

  void SetUp() override
  {
    *this = SharedData::instance();

    filter = std::make_shared<moving_average_filter::MultiChannelMovingAverageFilter<double>>();
  }

public:
  /**
   * @brief Shared pointer to the ROS2 node used for parameter handling.
   */
  rclcpp::Node::SharedPtr node;

  /**
   * @brief Pointer to the instance of the derived filter class.
   */
  std::shared_ptr<moving_average_filter::MultiChannelMovingAverageFilter<double>> filter;

  /**
   * @brief Maximum allowed difference to consider two numbers as equal.
   */
  const double EPSILON = 1e-6;

  /**
   * @brief Number of channels used in the multi channel filter.
   */
  const uint8_t NUMBER_OF_CHANNELS = 6;
};

/**
 * @brief Asserts the filter configuration is successful.
 */
TEST_F(MultiChannelMovingAverageFilterTest, testFilterConfiguration)
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
 * @brief Toggles the 6-channels input between {1, 2, 3, 4, 5, 6} and {0, 0, 0, 0, 0, 0}.
 * Checks that the mean is computed correctly over a 2-sized observation window, over 1e3 samples.
 */
TEST_F(MultiChannelMovingAverageFilterTest, testTwoSamplesWindowFilterDynamic)
{
  node->set_parameter(rclcpp::Parameter("number_of_observations", 2));

  ASSERT_TRUE(filter->configure(NUMBER_OF_CHANNELS,
                                "",                                                        // Prefix for parameters
                                "TwoSamplesWindowDynamicMultiChannelMovingAverageFilter",  // Filter name
                                node->get_node_logging_interface(),                        // Node logging interface
                                node->get_node_parameters_interface()                      // Node parameters interface
                                ));

  const std::vector<double> INITIAL_DATA_IN = { 1, 2, 3, 4, 5, 6 };

  // Since we are using a 2-sized window, the expected final value is the initial divided by two.
  const std::vector<double> FINAL_EXPECTED_VALUE = { 0.5, 1, 1.5, 2, 2.5, 3 };

  std::vector<double> expected_value;

  std::vector<double> data_in = INITIAL_DATA_IN;
  std::vector<double> data_out(NUMBER_OF_CHANNELS, 0);

  // Total number of samples to be processed
  const unsigned short int TOTAL_SAMPLES = 1e3;

  for (std::size_t i = 0; i < TOTAL_SAMPLES; i++)
  {
    // Update next input
    if (i % 2 == 1)
      std::fill(data_in.begin(), data_in.end(), 0);  // Toggle data_in for next iteration
    else
      data_in = INITIAL_DATA_IN;

    // Update expected mean value
    expected_value = (i == 0) ? INITIAL_DATA_IN : FINAL_EXPECTED_VALUE;

    // Call filter update
    filter->update(data_in, data_out);

    // Assert values are correct for each channel
    for (std::size_t j = 0; j < NUMBER_OF_CHANNELS; j++)
      ASSERT_NEAR(expected_value[j], data_out[j], EPSILON);
  }
}

/**
 * @brief Toggles the computation over 6-channels samples varying between {1, 1, 1, 1, 1, 1} and
 * {10, 10, 10, 10, 10, 10}, over an array with 10 samples and a observation window of 5 samples.
 */
TEST_F(MultiChannelMovingAverageFilterTest, testFiveSamplesWindowFilterDynamic)
{
  node->set_parameter(rclcpp::Parameter("number_of_observations", 5));

  ASSERT_TRUE(filter->configure(NUMBER_OF_CHANNELS,
                                "",                                                  // Prefix for parameters
                                "FiveSamplesWindowMultiChannelMovingAverageFilter",  // Filter name
                                node->get_node_logging_interface(),                  // Node logging interface
                                node->get_node_parameters_interface()                // Node parameters interface
                                ));

  std::vector<double> data_in = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

  // The expected cumulative value for the last 5 samples is {1, 3, 6, 10, 15, 20, 25, 30, 35, 40};

  // Dividing the cumulative value by the size of the filter window in the
  // MultiChannelMovingAverageFilter yields the expected value.
  std::vector<double> expected = { 1, 1.5, 2, 2.5, 3, 4, 5, 6, 7, 8 };

  // Total number of samples to be processed
  const std::size_t TOTAL_SAMPLES = std::size(data_in);

  // Variable for the filter output
  std::vector<double> data_out(NUMBER_OF_CHANNELS);

  std::vector<double> data_in_vector(NUMBER_OF_CHANNELS, 0.0);
  std::vector<double> expected_value_vector(NUMBER_OF_CHANNELS, 0.0);

  for (std::size_t i = 0; i < TOTAL_SAMPLES; i++)
  {
    data_in_vector = std::vector<double>(NUMBER_OF_CHANNELS, data_in[i]);

    expected_value_vector = std::vector<double>(NUMBER_OF_CHANNELS, expected[i]);

    filter->update(data_in_vector, data_out);

    // Assert values are correct for each channel
    for (std::size_t j = 0; j < NUMBER_OF_CHANNELS; j++)
      ASSERT_NEAR(expected_value_vector[j], data_out[j], EPSILON);
  }
}

/**
 * @brief Test the mean computation over a 5000-sized observation window, in an array with 1e5 samples.
 * The observed value is static.
 */
TEST_F(MultiChannelMovingAverageFilterTest, testFiveThousandSamplesWindowFilterStatic)
{
  node->set_parameter(rclcpp::Parameter("number_of_observations", 5000));

  ASSERT_TRUE(filter->configure(NUMBER_OF_CHANNELS,
                                "",                                                          // Prefix for parameters
                                "FiveThousandSamplesWindowMultiChannelMovingAverageFilter",  // Filter name
                                node->get_node_logging_interface(),                          // Node logging interface
                                node->get_node_parameters_interface()                        // Node parameters interface
                                ));

  // Total number of samples to be processed
  const unsigned int TOTAL_SAMPLES = 1e5;

  // Value that will be replicated in a 6-sized vector and fed at each input
  const unsigned short int DATA_IN = 42;

  // Input vector
  std::vector<double> data_in_vector(NUMBER_OF_CHANNELS, DATA_IN);

  // The filter expects a data_out vector of same size as the input.
  std::vector<double> data_out_vector(NUMBER_OF_CHANNELS, 0);

  rclcpp::Time begin = rclcpp::Clock().now();

  for (std::size_t i = 0; i < TOTAL_SAMPLES; i++)
  {
    filter->update(data_in_vector, data_out_vector);

    // Assert values are correct for each channel
    for (std::size_t j = 0; j < NUMBER_OF_CHANNELS; j++)
      ASSERT_NEAR(data_in_vector[j], data_out_vector[j], EPSILON);
  }
}

/**
 * @brief Instantiate the MultiChannelMeanFilter and the MultiChannelMovingAverageFilter and assert that the output from both is the same.
 */
TEST_F(MultiChannelMovingAverageFilterTest, testCompareMultiChannelMovingAverageAndMultiChannelMeanFiltersOutput)
{
  // Instantiate the multi-channel mean filter
  std::shared_ptr<filters::MultiChannelFilterBase<double>> mean_filter = std::make_shared<filters::MultiChannelMeanFilter<double>>();
  node->set_parameter(rclcpp::Parameter("number_of_observations", 5000));

  ASSERT_TRUE(mean_filter->configure(NUMBER_OF_CHANNELS,
                                     "",                                                 // Prefix for parameters
                                     "FiveThousandSamplesWindowMultiChannelMeanFilter",  // Filter name
                                     node->get_node_logging_interface(),                 // Node logging interface
                                     node->get_node_parameters_interface()               // Node parameters interface
                                     ));

  ASSERT_TRUE(filter->configure(NUMBER_OF_CHANNELS,
                                "",                                                          // Prefix for parameters
                                "FiveThousandSamplesWindowMultiChannelMovingAverageFilter",  // Filter name
                                node->get_node_logging_interface(),                          // Node logging interface
                                node->get_node_parameters_interface()                        // Node parameters interface
                                ));

  // Total number of samples to be processed
  const unsigned long TOTAL_SAMPLES = 1e5;

  // The filter expects a data_out vector of same size as the input.
  std::vector<double> data_out_vector_mean(NUMBER_OF_CHANNELS, 0);
  std::vector<double> data_out_vector_moving_average(NUMBER_OF_CHANNELS, 0);

  unsigned int seed = time(nullptr);
  srand48(seed);
  for (std::size_t i = 0; i < TOTAL_SAMPLES; i++)
  {
    // Define input
    std::vector<double> data_in_vector(NUMBER_OF_CHANNELS, 0);

    // Calculate a random double number in range [-42;42]
    for (std::size_t i = 0; i < NUMBER_OF_CHANNELS; i++)
      data_in_vector[i] = (drand48() - 0.5) * 2 * 42;

    // Update filters
    mean_filter->update(data_in_vector, data_out_vector_mean);
    filter->update(data_in_vector, data_out_vector_moving_average);

    // Assert outputs are equal
    for (std::size_t j = 0; j < NUMBER_OF_CHANNELS; j++)
      ASSERT_NEAR(data_out_vector_mean[j], data_out_vector_moving_average[j], EPSILON) << "Test failed with seed " << std::to_string(seed);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
