/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_gravity_compensation_variable_orientation.cpp
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Mar 4, 2025
 *
 * This class provides an integration test for gravity compensation in
 * the case of variable robot orientation.
 *
 * -------------------------------------------------------------------
 */

#include <gtest/gtest.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <filters/filter_chain.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using namespace std::chrono_literals;

// This class shares parameters and data across all tests
class SharedData
{
  friend class GravityCompensationVariableOrientationTest;

  // Private members
  std::shared_ptr<rclcpp::Node> node_;
  std::string bag_filename_;
  std::string bag_package_;

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

    // Load parameters
    ASSERT_TRUE(node_->get_parameter("bag_package", bag_package_));
    ASSERT_TRUE(node_->get_parameter("bag_filename", bag_filename_));
  }

public:
  static const SharedData& instance()
  {
    static SharedData instance;
    return instance;
  }
  static void release() {}
};

class GravityCompensationVariableOrientationTest : public ::testing::Test
{
protected:
  void operator=(const SharedData& data)
  {
    node = data.node_;
    bag_filename = data.bag_filename_;
    bag_package = data.bag_package_;
  }

  void SetUp() override
  {
    *this = SharedData::instance();

    // Initialize the filter chain
    filter_chain = std::make_shared<filters::MultiChannelFilterChain<double>>("double");

    // Read trajectory from bag file
    std::string bagfile_path = ament_index_cpp::get_package_share_directory(bag_package) + bag_filename;

    rclcpp::Serialization<trajectory_msgs::msg::JointTrajectory> serialization;
    RCLCPP_INFO(node->get_logger(), "Opening '%s'", bagfile_path.c_str());
    rosbag2_cpp::Reader reader;
    reader.open(bagfile_path);

    while (reader.has_next())
    {
      rosbag2_storage::SerializedBagMessageSharedPtr message = reader.read_next();
      rclcpp::SerializedMessage serialized_message(*message->serialized_data);
      serialization.deserialize_message(&serialized_message, &trajectory_msg);
      RCLCPP_INFO(node->get_logger(), "Trajectory with %ld waypoints read.", trajectory_msg.points.size());
    }

    reader.close();

    // Create a subscriber for the force/torque sensor mesurements
    subscriber = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/force_torque_sensor_broadcaster/wrench", rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&GravityCompensationVariableOrientationTest::fts_callback_, this, std::placeholders::_1));

    executor.add_node(node);

    // Create a trajectory publisher
    trajectory_publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);

    // Create a buffer for managing tf transformations
    tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  }

  /**
   * @brief Callback function for force/torque messages.
   *
   * @param msg The incoming message containing force and torque measurements.
   */
  void fts_callback_(const geometry_msgs::msg::WrenchStamped& msg)
  {
    wrench = msg;
  }

public:
  /**
   * @brief Get the current orientation of the base frame with respect to the f/t sensor frame.
   */
  geometry_msgs::msg::Quaternion get_ft_sensor_to_base_orientation(const rclcpp::Time& time) const
  {
    geometry_msgs::msg::TransformStamped transform_msg = tf_buffer->lookupTransform("ft_sensing_frame", "world", time, tf2::durationFromSec(0.005));
    return transform_msg.transform.rotation;
  }

  /**
   * @brief Shared pointer to the ROS2 node.
   */
  rclcpp::Node::SharedPtr node;

  /**
   * @brief Pointer to the instance of the derived filter class.
   */
  std::shared_ptr<filters::MultiChannelFilterChain<double>> filter_chain;

  /**
   * @brief Number of channels used in the filter chain.
   */
  const uint8_t NUMBER_OF_CHANNELS = 11;

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
   * @brief Buffer for managing tf transformations.
   */
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;

  /**
   * @brief Listener for subscribing to tf transformations.
   */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  /**
   * @brief Force/torque measurement message.
   */
  geometry_msgs::msg::WrenchStamped wrench;

  /**
   * @brief Bag filename for reading the trajectory.
   */
  std::string bag_filename;

  /**
   * @brief Bag file package name.
   */
  std::string bag_package;

  /**
   * @brief Trajectory message for moving the robot to specific orientations.
   */
  trajectory_msgs::msg::JointTrajectory trajectory_msg;
};

/**
 * @brief Unit test for verifying the gravity compensation for different robot orientations.
 */
TEST_F(GravityCompensationVariableOrientationTest, testGravityCompensation)
{
  // Initialize the filter input and output vectors
  std::vector<double> filter_chain_output(NUMBER_OF_CHANNELS, 0.0);
  std::vector<double> filter_chain_input(NUMBER_OF_CHANNELS, 0.0);

  ASSERT_TRUE(filter_chain->configure(NUMBER_OF_CHANNELS,
                                      "filter_chain",                        // Prefix for parameters
                                      node->get_node_logging_interface(),    // Node logging interface
                                      node->get_node_parameters_interface()  // Node parameters interface
                                      ));

  RCLCPP_INFO(node->get_logger(), "Chain configured");

  rclcpp::Time start_time = node->now();
  rclcpp::Duration elapsed = node->now() - start_time;
  auto last_time = trajectory_msg.points.back().time_from_start;
  rclcpp::Duration simulation_duration(last_time.sec + 5.0, last_time.nanosec);

  RCLCPP_INFO(node->get_logger(), "Starting gravity compensation test");

  auto update_filter = [&]()
  {
    // Spin the node to process the force/torque sensor measurements
    executor.spin_some();

    // Get the sensor-to-base orientation quaternion
    geometry_msgs::msg::Quaternion quaternion = get_ft_sensor_to_base_orientation(rclcpp::Time(wrench.header.stamp));

    // Prepare input for the filter chain
    filter_chain_input[0] = wrench.wrench.force.x + SENSOR_BIAS[0];
    filter_chain_input[1] = wrench.wrench.force.y + SENSOR_BIAS[1];
    filter_chain_input[2] = wrench.wrench.force.z + SENSOR_BIAS[2];
    filter_chain_input[3] = wrench.wrench.torque.x + SENSOR_BIAS[3];
    filter_chain_input[4] = wrench.wrench.torque.y + SENSOR_BIAS[4];
    filter_chain_input[5] = wrench.wrench.torque.z + SENSOR_BIAS[5];
    filter_chain_input[6] = quaternion.w;
    filter_chain_input[7] = quaternion.x;
    filter_chain_input[8] = quaternion.y;
    filter_chain_input[9] = quaternion.z;

    // Update the filter chain
    filter_chain->update(filter_chain_input, filter_chain_output);
  };

  while (filter_chain_output[NUMBER_OF_CHANNELS - 1] == 0.0)
  {
    update_filter();
  }

  // Publish the trajectory message
  trajectory_publisher->publish(trajectory_msg);

  while (elapsed < simulation_duration)
  {
    update_filter();

    // Check the gravity compensation
    for (size_t j = 0; j < 6; j++)
    {
      ASSERT_NEAR(filter_chain_output[j], 0.0, COMPENSATION_ERROR_THRESHOLD);
    }

    elapsed = node->now() - start_time;
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  SharedData::release();
  return result;
}
