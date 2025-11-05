/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_moving_average_filter.cpp
 * Author:  Alessio Coone
 * Org.:    UNISA
 * Date:    Oct 28, 2024
 *
 * This class provides a set of tests for the MovingAverageFilter
 * class.
 *
 * -------------------------------------------------------------------
 */

#include <gtest/gtest.h>
#include <filters/mean.hpp>
#include "moving_average_filter/moving_average_filter.hpp"

// This class shares parameters and data across all tests
class SharedData
{
  friend class MovingAverageFilterTest;

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
    node_options.arguments({ "--ros-args", "-r", "__node:=moving_average_filter_test" });
    node_ = rclcpp::Node::make_shared("moving_average_filter_test", node_options);

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

class MovingAverageFilterTest : public ::testing::Test
{
protected:
  void operator=(const SharedData& data)
  {
    node = data.node_;
  }

  void SetUp() override
  {
    *this = SharedData::instance();

    filter = std::make_shared<moving_average_filter::MovingAverageFilter<double>>();
  }

public:
  /**
   * @brief Shared pointer to the ROS2 node used for parameter handling.
   */
  std::shared_ptr<rclcpp::Node> node;

  /**
   * @brief Pointer to the instance of the derived filter class.
   */
  std::shared_ptr<moving_average_filter::MovingAverageFilter<double>> filter;

  /**
   * @brief Maximum allowed difference to consider two numbers as equal.
   */
  const double EPSILON = 1e-6;
};

/**
 * @brief Fixture that asserts the filter configuration is successful.
 */
TEST_F(MovingAverageFilterTest, testFilterConfiguration)
{
  node->set_parameter(rclcpp::Parameter("number_of_observations", 2));

  ASSERT_TRUE(filter->configure("",                                    // Prefix for parameters
                                "MovingAverageFilter",                 // Filter name
                                node->get_node_logging_interface(),    // Node logging interface
                                node->get_node_parameters_interface()  // Node parameters interface
                                ));
}

/**
 * @brief Fixture that asserts the filter configuration is unsuccessful.
 */
TEST_F(MovingAverageFilterTest, testFilterConfigurationFailure)
{
  node->set_parameter(rclcpp::Parameter("number_of_observations", 0));

  ASSERT_FALSE(filter->configure("",                                    // Prefix for parameters
                                 "MovingAverageFilter",                 // Filter name
                                 node->get_node_logging_interface(),    // Node logging interface
                                 node->get_node_parameters_interface()  // Node parameters interface
                                 ));
}

/**
 * @brief Toggles the input between 0 and 1 and checks that the mean over a 2-sized observation
 * window is computed correctly, over 1e3 samples.
 */
TEST_F(MovingAverageFilterTest, testTwoSamplesWindowFilterDynamic)
{
  node->set_parameter(rclcpp::Parameter("number_of_observations", 2));

  ASSERT_TRUE(filter->configure("",                                     // Prefix for parameters
                                "TwoSamplesWindowMovingAverageFilter",  // Filter name
                                node->get_node_logging_interface(),     // Node logging interface
                                node->get_node_parameters_interface()   // Node parameters interface
                                ));

  double data_in = 1;
  double data_out = 0;
  const unsigned long TOTAL_SAMPLES = 1e3;

  for (std::size_t i = 0; i < TOTAL_SAMPLES; i++)
  {
    filter->update(data_in, data_out);
    data_in = !data_in;  // Toggle data_in for next iteration

    double expected_value = (i == 0) ? 1 : 0.5;
    ASSERT_NEAR(expected_value, data_out, EPSILON);
  }
}

/**
 * @brief Test the mean computation over a 5-sized observation window, in an array with 10 samples.
 * The observed value is static.
 */
TEST_F(MovingAverageFilterTest, testFiveSamplesWindowFilterStatic)
{
  node->set_parameter(rclcpp::Parameter("number_of_observations", 5));

  ASSERT_TRUE(filter->configure("",                                      // Prefix for parameters
                                "FiveSamplesWindowMovingAverageFilter",  // Filter name
                                node->get_node_logging_interface(),      // Node logging interface
                                node->get_node_parameters_interface()    // Node parameters interface
                                ));

  // Input sample fed to the filter at each step
  const std::vector<double> DATA_IN = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

  // Dividing the cumulative value by N yields the expected value:
  const std::vector<double> EXPECTED = { 1, 1.5, 2, 2.5, 3, 4, 5, 6, 7, 8 };

  // Total number of samples to be processed
  const unsigned short TOTAL_SAMPLES = 10;

  // Variable for the filter output
  double data_out;

  for (std::size_t i = 0; i < TOTAL_SAMPLES; i++)
  {
    filter->update(DATA_IN[i], data_out);
    ASSERT_NEAR(EXPECTED[i], data_out, EPSILON);
  }
}

/**
 * @brief Test the mean computation over a 5000-sized observation window, in an array with 1e6 samples.
 * The observed value is static.
 */
TEST_F(MovingAverageFilterTest, testFiveThousandSamplesWindowFilterStatic)
{
  node->set_parameter(rclcpp::Parameter("number_of_observations", 5000));

  ASSERT_TRUE(filter->configure("",                                              // Prefix for parameters
                                "FiveThousandSamplesWindowMovingAverageFilter",  // Filter name
                                node->get_node_logging_interface(),              // Node logging interface
                                node->get_node_parameters_interface()            // Node parameters interface
                                ));

  // Total number of samples to be processed
  const unsigned long TOTAL_SAMPLES = 1e6;

  // Value that will be fed at each input
  const double DATA_IN = 42;

  // Variable for the filter output
  double data_out;

  rclcpp::Time begin = rclcpp::Clock().now();

  for (std::size_t i = 0; i < TOTAL_SAMPLES; i++)
  {
    filter->update(DATA_IN, data_out);

    ASSERT_NEAR(DATA_IN, data_out, EPSILON);
  }
}

/**
 * @brief Instantiate the MeanFilter and the MovingAverageFilter filters and assert that their output is the same.
 */
TEST_F(MovingAverageFilterTest, testCompareMovingAverageAndMeanFiltersOutput)
{
  // Instantiate the mean filter
  std::shared_ptr<filters::FilterBase<double>> mean_filter = std::make_shared<filters::MeanFilter<double>>();
  node->set_parameter(rclcpp::Parameter("number_of_observations", 5000));

  ASSERT_TRUE(mean_filter->configure("",                                     // Prefix for parameters
                                     "FiveThousandSamplesWindowMeanFilter",  // Filter name
                                     node->get_node_logging_interface(),     // Node logging interface
                                     node->get_node_parameters_interface()   // Node parameters interface
                                     ));

  ASSERT_TRUE(filter->configure("",                                              // Prefix for parameters
                                "FiveThousandSamplesWindowMovingAverageFilter",  // Filter name
                                node->get_node_logging_interface(),              // Node logging interface
                                node->get_node_parameters_interface()            // Node parameters interface
                                ));

  // Total number of samples to be processed
  const unsigned long TOTAL_SAMPLES = 1e5;

  double data_out_mean = 0.0;
  double data_out_moving_average = 0.0;

  unsigned int seed = time(nullptr);
  srand48(seed);
  for (std::size_t i = 0; i < TOTAL_SAMPLES; i++)
  {
    // Calculate a random double number in range [-42;42]
    double data_in = (drand48() - 0.5) * 2 * 42;

    // Update filters
    mean_filter->update(data_in, data_out_mean);
    filter->update(data_in, data_out_moving_average);

    // Assert outputs are equal
    ASSERT_NEAR(data_out_mean, data_out_moving_average, EPSILON) << "Test failed with seed " << std::to_string(seed);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
