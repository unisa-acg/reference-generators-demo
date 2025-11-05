/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_gravity_compensation_fixed_orientation.cpp
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Mar 4, 2025
 *
 * This class provides an integration test for gravity compensation in
 * the case of fixed robot orientation.
 *
 * -------------------------------------------------------------------
 */

#include <gtest/gtest.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "unbiasing_filter/unbiasing_filter.hpp"

using namespace std::chrono_literals;

// This class shares parameters and data across all tests
class SharedData
{
  friend class GravityCompensationFixedOrientationTest;

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
    node_options.arguments({ "--ros-args", "-r", "__node:=gravity_compensation_test" });
    node_ = rclcpp::Node::make_shared("gravity_compensation_test", node_options);
  }

public:
  static const SharedData& instance()
  {
    static SharedData instance;
    return instance;
  }
};

class GravityCompensationFixedOrientationTest : public ::testing::Test
{
protected:
  void operator=(const SharedData& data)
  {
    node = data.node_;
  }

  void SetUp() override
  {
    *this = SharedData::instance();

    // Create a subscriber for the force/torque sensor mesurements
    subscriber = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/force_torque_sensor_broadcaster/wrench", rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&GravityCompensationFixedOrientationTest::fts_callback_, this, std::placeholders::_1));

    executor.add_node(node);

    is_simulation_active = false;

    // Create a trajectory publisher
    trajectory_publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);

    // Define the trajectory message for moving the robot to specific orientations
    trajectory_msg.points.resize(1);

    // Define the joint names for the robot's trajectory
    trajectory_msg.joint_names = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };

    // Set the time for the trajectory execution
    trajectory_msg.points[0].time_from_start.sec = 1.0;
    trajectory_msg.points[0].time_from_start.nanosec = 0.0;

    filter = std::make_shared<unbiasing_filter::UnbiasingFilter<double>>();
  }

  /**
   * @brief Callback function for force/torque messages.
   *
   * @param msg The incoming message containing force and torque measurements.
   */
  void fts_callback_(const geometry_msgs::msg::WrenchStamped& msg)
  {
    wrench = msg;
    is_simulation_active = true;
  }

public:
  /**
   * @brief Shared pointer to the ROS2 node.
   */
  rclcpp::Node::SharedPtr node;

  /**
   * @brief Pointer to the instance of the derived filter class.
   */
  std::shared_ptr<unbiasing_filter::UnbiasingFilter<double>> filter;

  /**
   * @brief Number of channels used in the filter.
   */
  const uint8_t NUMBER_OF_CHANNELS = 7;

  /**
   * @brief Define the error threshold for gravity compensation.
   */
  const double COMPENSATION_ERROR_THRESHOLD = 0.1;

  /**
   * @brief Fictitious force/torque sensor bias.
   */
  const std::vector<double> SENSOR_BIAS = { 1.0, 1.5, 5.0, 0.01, 0.017, 0.6 };

  /**
   * @brief Publisher for sending joint trajectories to the robot.
   */
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher;

  /**
   * @brief Subscriber for receiving the force/torque sensor measurements.
   */
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber;

  /**
   * @brief Executor for spinning the node.
   */
  rclcpp::executors::SingleThreadedExecutor executor;

  /**
   * @brief Flag indicating whether the simulation is active.
   */
  bool is_simulation_active;

  /**
   * @brief Force/torque measurement message.
   */
  geometry_msgs::msg::WrenchStamped wrench;

  /**
   * @brief Trajectory message for moving the robot to specific orientations.
   */
  trajectory_msgs::msg::JointTrajectory trajectory_msg;

  /**
   * @brief Vector of trajectory points for testing gravity compensation along Z, Y, and X axes.
   */
  const std::vector<std::vector<double>> TRAJECTORY_POINTS = {
    { 0.0, -1.57, 0.0, 0.0, 1.57, 0.0 },   // Position for testing gravity compensation along Z-axis
    { 0.0, -1.57, 0.0, -1.57, 0.0, 0.0 },  // Position for testing gravity compensation along Y-axis
    { 0.0, -1.57, 0.0, 0.0, 0.0, 0.0 }     // Position for testing gravity compensation along X-axis
  };
};

/**
 * @brief Unit test for verifying the gravity compensation for a fixed robot orientation.
 * The gravity compensation is validated for three different fixed orientations.
 */
TEST_F(GravityCompensationFixedOrientationTest, testGravityCompensationAlongZYXAxes)
{
  for (size_t i = 0; i < TRAJECTORY_POINTS.size(); i++)
  {
    // Initialize the filter input and output vectors
    std::vector<double> filter_output(NUMBER_OF_CHANNELS, 0.0);
    std::vector<double> filter_input(NUMBER_OF_CHANNELS, 0.0);

    // Set the trajectory point
    trajectory_msg.points[0].positions = TRAJECTORY_POINTS[i];

    // Publish the trajectory message
    trajectory_publisher->publish(trajectory_msg);

    // Wait for the robot to execute the trajectory
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Reset the filter
    if (i == 0)
    {
      ASSERT_TRUE(filter->configure(NUMBER_OF_CHANNELS,
                                    "",                                    // Prefix for parameters
                                    "UnbiasingFilter",                     // Filter name
                                    node->get_node_logging_interface(),    // Node logging interface
                                    node->get_node_parameters_interface()  // Node parameters interface
                                    ));
    }
    else
    {
      ASSERT_TRUE(filter->configure());
    }

    while (filter_output[NUMBER_OF_CHANNELS - 1] == 0.0)
    {
      // Spin the node to process the force/torque sensor measurements
      executor.spin_some();

      ASSERT_TRUE(is_simulation_active) << "Simulation is not active. No force/torque sensor data received.";

      // Prepare input for the unbiasing filter
      filter_input[0] = wrench.wrench.force.x + SENSOR_BIAS[0];
      filter_input[1] = wrench.wrench.force.y + SENSOR_BIAS[1];
      filter_input[2] = wrench.wrench.force.z + SENSOR_BIAS[2];
      filter_input[3] = wrench.wrench.torque.x + SENSOR_BIAS[3];
      filter_input[4] = wrench.wrench.torque.y + SENSOR_BIAS[4];
      filter_input[5] = wrench.wrench.torque.z + SENSOR_BIAS[5];
      filter_input[6] = 0.0;

      filter->update(filter_input, filter_output);
    }

    // Check the gravity compensation
    for (size_t j = 0; j < 6; j++)
    {
      EXPECT_NEAR(filter_output[j], 0.0, COMPENSATION_ERROR_THRESHOLD);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
