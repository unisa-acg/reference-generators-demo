/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_admittance_filter.cpp
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    May 11, 2025
 *
 * Unit test for AdmittanceFilter class.
 * The ground truth is obtained with MATLAB by generating in Simulink
 * the expected output. Refer to the package Readme for more
 * information.
 *
 * -------------------------------------------------------------------
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <pluginlib/class_loader.hpp>
#include <xxx_common_libraries/message_utilities.hpp>
#include <xxx_control_msgs/msg/task_space_point.hpp>
#include <interaction_filter_base/interaction_filter_base.hpp>
#include "admittance_filter/admittance_filter.hpp"

namespace interaction_filters
{

struct AdmittanceFilterParameters
{
  int filter_order;
  std::string input_type;
};

// This class shares parameters and data across all tests
class SharedData
{
  typedef pluginlib::ClassLoader<interaction_filters::InteractionFilterBase> FilterLoader;

  friend class AdmittanceFilterTest;

  // Private members
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<FilterLoader> filter_loader_;

  SharedData(const SharedData&) = delete;  // this is a singleton
  SharedData()
  {
    initialize();
  }

  void initialize()
  {
    // Instantiate the node
    node_ = rclcpp::Node::make_shared("admittance_filter_test");
    node_->declare_parameter("admittance_filter.mass", std::vector<double>{ 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 });
    node_->declare_parameter("admittance_filter.damping_ratio",
                             std::vector<double>{ 1.58113883, 1.315587029, 1.58113883, 1.58113883, 1.58113883, 1.58113883 });
    node_->declare_parameter("admittance_filter.stiffness", std::vector<double>{ 900.0, 1300.0, 100.0, 100.0, 100.0, 100.0 });
    node_->declare_parameter("admittance_filter.order", 2);
    node_->declare_parameter("admittance_filter.compliant_axis", std::vector<bool>{ true, true, true, true, true, true });

    // Initialize the filter class loader
    filter_loader_ = std::make_shared<FilterLoader>("interaction_filter_base", "interaction_filters::InteractionFilterBase");
    ASSERT_TRUE(bool(filter_loader_)) << "Failed to instantiate ClassLoader<FilterLoader>";
  }

public:
  pluginlib::UniquePtr<interaction_filters::InteractionFilterBase> createUniqueInstance(const std::string& name) const
  {
    return filter_loader_->createUniqueInstance(name);
  }

  static const SharedData& instance()
  {
    static SharedData instance;
    return instance;
  }
  static void release()
  {
    SharedData& shared = const_cast<SharedData&>(instance());
    shared.filter_loader_.reset();
  }
};

class AdmittanceFilterTest : public ::testing::TestWithParam<AdmittanceFilterParameters>
{
public:
  void SetUp() override;
  void TearDown() override
  {
    interaction_filter_.reset();
  }

  static void TearDownTestSuite()
  {
    // Called once at the end of all tests
    interaction_filters::SharedData::release();
  }

protected:
  void operator=(const SharedData& data)
  {
    node_ = data.node_;
  }

  /**
   * @brief Shared pointer to the ROS2 node.
   */
  rclcpp::Node::SharedPtr node_;

  /**
   * @brief Input message for the filter.
   */
  std::vector<geometry_msgs::msg::WrenchStamped> filter_input_;

  /**
   * @brief Output message of the filter.
   */
  std::vector<std_msgs::msg::Float64MultiArray> filter_output_;

  /**
   * @brief Pointer to the instance of the admittance filter.
   */
  std::shared_ptr<interaction_filters::InteractionFilterBase> interaction_filter_;
};

void AdmittanceFilterTest::SetUp()
{
  *this = SharedData::instance();

  // Create the AdmittanceFilter instance
  ASSERT_NO_THROW({ interaction_filter_ = SharedData::instance().createUniqueInstance("interaction_filters/AdmittanceFilter"); });
  ASSERT_TRUE(bool(interaction_filter_)) << "Failed to load plugin: interaction_filters/AdmittanceFilter";

  // Initialize the parameters
  const AdmittanceFilterParameters& params = GetParam();

  // Set the parameters
  int filter_order = params.filter_order;
  node_->set_parameter(rclcpp::Parameter("admittance_filter.order", filter_order));

  // Initialize the filter
  ASSERT_TRUE(interaction_filter_->initialize(node_->get_node_parameters_interface(), node_->get_node_logging_interface(), "admittance_filter"));

  // Read trajectory from bag file
  std::string bag_filename = params.input_type + "_response_order_" + std::to_string(filter_order);
  std::string bagfile_path =
      ament_index_cpp::get_package_share_directory("admittance_filter") + "/test/bagfiles/" + bag_filename + "/" + bag_filename + "_0.db3";

  RCLCPP_INFO(node_->get_logger(), "Opening '%s'", bagfile_path.c_str());
  rosbag2_cpp::Reader reader;
  reader.open(bagfile_path);

  while (reader.has_next())
  {
    rosbag2_storage::SerializedBagMessageSharedPtr message = reader.read_next();

    if (message->topic_name == "/admittance_filter_input")
    {
      rclcpp::Serialization<geometry_msgs::msg::WrenchStamped> serialization;
      rclcpp::SerializedMessage serialized_message(*message->serialized_data);
      geometry_msgs::msg::WrenchStamped msg;
      serialization.deserialize_message(&serialized_message, &msg);
      filter_input_.push_back(msg);
    }

    if (message->topic_name == "/admittance_filter_output")
    {
      rclcpp::Serialization<std_msgs::msg::Float64MultiArray> serialization;
      rclcpp::SerializedMessage serialized_message(*message->serialized_data);
      std_msgs::msg::Float64MultiArray msg;
      serialization.deserialize_message(&serialized_message, &msg);
      filter_output_.push_back(msg);
    }
  }

  reader.close();
}

TEST_P(AdmittanceFilterTest, TestAdmittanceFilter)
{
  const double threshold = 1e-7;  // Tolerance for the test

  rclcpp::Duration previous_time = rclcpp::Duration(0, 0);
  for (std::size_t i = 0; i < filter_input_.size(); ++i)
  {
    // Set the filter input
    interaction_filters::Vector6d delta_h;
    tf2::fromMsg(filter_input_[i].wrench, delta_h);

    // Set the reference for the admittance filter
    xxx_control_msgs::msg::TaskSpacePoint admittance_filter_input;
    admittance_filter_input.pose = geometry_msgs::msg::Pose();
    admittance_filter_input.twist = geometry_msgs::msg::Twist();
    admittance_filter_input.acceleration = geometry_msgs::msg::Accel();
    tf2::toMsg(delta_h, admittance_filter_input.wrench);

    // Compute the filter output
    xxx_control_msgs::msg::TaskSpacePoint admittance_filter_output;
    rclcpp::Duration current_time(filter_input_[i].header.stamp.sec, filter_input_[i].header.stamp.nanosec);
    ASSERT_TRUE(interaction_filter_->update(admittance_filter_input, current_time - previous_time, admittance_filter_output));

    // Compare computed output with expected output (within a certain threshold)
    // Pose
    geometry_msgs::msg::Pose expected_pose;
    tf2::toMsg(interaction_filters::Vector6d(filter_output_[i].data.data()), expected_pose);
    EXPECT_NEAR(admittance_filter_output.pose.position.x, expected_pose.position.x, threshold);
    EXPECT_NEAR(admittance_filter_output.pose.position.y, expected_pose.position.y, threshold);
    EXPECT_NEAR(admittance_filter_output.pose.position.z, expected_pose.position.z, threshold);
    EXPECT_NEAR(admittance_filter_output.pose.orientation.x, expected_pose.orientation.x, threshold);
    EXPECT_NEAR(admittance_filter_output.pose.orientation.y, expected_pose.orientation.y, threshold);
    EXPECT_NEAR(admittance_filter_output.pose.orientation.z, expected_pose.orientation.z, threshold);
    EXPECT_NEAR(admittance_filter_output.pose.orientation.w, expected_pose.orientation.w, threshold);

    // Twist
    EXPECT_NEAR(admittance_filter_output.twist.linear.x, filter_output_[i].data[6], threshold);
    EXPECT_NEAR(admittance_filter_output.twist.linear.y, filter_output_[i].data[7], threshold);
    EXPECT_NEAR(admittance_filter_output.twist.linear.z, filter_output_[i].data[8], threshold);
    EXPECT_NEAR(admittance_filter_output.twist.angular.x, filter_output_[i].data[9], threshold);
    EXPECT_NEAR(admittance_filter_output.twist.angular.y, filter_output_[i].data[10], threshold);
    EXPECT_NEAR(admittance_filter_output.twist.angular.z, filter_output_[i].data[11], threshold);

    // Acceleration
    EXPECT_NEAR(admittance_filter_output.acceleration.linear.x, filter_output_[i].data[12], threshold);
    EXPECT_NEAR(admittance_filter_output.acceleration.linear.y, filter_output_[i].data[13], threshold);
    EXPECT_NEAR(admittance_filter_output.acceleration.linear.z, filter_output_[i].data[14], threshold);
    EXPECT_NEAR(admittance_filter_output.acceleration.angular.x, filter_output_[i].data[15], threshold);
    EXPECT_NEAR(admittance_filter_output.acceleration.angular.y, filter_output_[i].data[16], threshold);
    EXPECT_NEAR(admittance_filter_output.acceleration.angular.z, filter_output_[i].data[17], threshold);

    previous_time = current_time;
  }
}

AdmittanceFilterParameters getAdmittanceFilterOrder0RampParameters()
{
  AdmittanceFilterParameters params;
  params.filter_order = 0;
  params.input_type = "ramp";
  return params;
}

AdmittanceFilterParameters getAdmittanceFilterOrder1RampParameters()
{
  AdmittanceFilterParameters params;
  params.filter_order = 1;
  params.input_type = "ramp";
  return params;
}

AdmittanceFilterParameters getAdmittanceFilterOrder2RampParameters()
{
  AdmittanceFilterParameters params;
  params.filter_order = 2;
  params.input_type = "ramp";
  return params;
}
AdmittanceFilterParameters getAdmittanceFilterOrder0StepParameters()
{
  AdmittanceFilterParameters params;
  params.filter_order = 0;
  params.input_type = "step";
  return params;
}
AdmittanceFilterParameters getAdmittanceFilterOrder1StepParameters()
{
  AdmittanceFilterParameters params;
  params.filter_order = 1;
  params.input_type = "step";
  return params;
}
AdmittanceFilterParameters getAdmittanceFilterOrder2StepParameters()
{
  AdmittanceFilterParameters params;
  params.filter_order = 2;
  params.input_type = "step";
  return params;
}

INSTANTIATE_TEST_SUITE_P(AdmittanceFilterTests, AdmittanceFilterTest,
                         ::testing::Values(getAdmittanceFilterOrder0RampParameters(), getAdmittanceFilterOrder1RampParameters(),
                                           getAdmittanceFilterOrder2RampParameters(), getAdmittanceFilterOrder0StepParameters(),
                                           getAdmittanceFilterOrder1StepParameters(), getAdmittanceFilterOrder2StepParameters()));

}  // namespace interaction_filters

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
