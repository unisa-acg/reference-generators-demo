/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   test_kinematics.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Mar 26, 2025
 *
 * This is a test for the kinematics library.
 *
 * -------------------------------------------------------------------
 */

// Gtest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Dense>

// ROS2
#include <rclcpp/rclcpp.hpp>

// PluginLib
#include <pluginlib/class_loader.hpp>

// Kinematics Interface
#include <kinematics_interface/kinematics_interface.hpp>

// Task Space Point
#include <acg_control_msgs/msg/task_space_point.hpp>

// Message utilities
#include "acg_common_libraries/message_utilities.hpp"

// File under tests
#include "acg_common_libraries/kinematics.hpp"

// This class shares parameters and data across all tests
class SharedData
{
  friend class KinematicsTest;

protected:
  typedef pluginlib::ClassLoader<kinematics_interface::KinematicsInterface> KinematicsInterfaceLoader;
  std::shared_ptr<rclcpp::Node> node_;
  std::string plugin_package_;
  std::string kinematics_interface_plugin_name_;
  std::string robot_description_;
  std::string tip_link_;
  std::unique_ptr<KinematicsInterfaceLoader> kinematics_loader_;

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
    node_ = rclcpp::Node::make_shared("test_kinematics_node", node_options);

    // Load parameters
    EXPECT_TRUE(node_->get_parameter("plugin_package", plugin_package_));
    EXPECT_TRUE(node_->get_parameter("kinematics_interface_plugin_name", kinematics_interface_plugin_name_));

    // The kinematics_interface requires the node to have the `robot_description` parameter.
    EXPECT_TRUE(node_->get_parameter("robot_description", robot_description_));
    EXPECT_TRUE(node_->get_parameter("tip_link", tip_link_));

    // Initialize inverse dynamics solver class loader
    kinematics_loader_ = std::make_unique<KinematicsInterfaceLoader>(plugin_package_, "kinematics_interface::KinematicsInterface");
    ASSERT_TRUE(bool(kinematics_loader_)) << "Failed to instantiate ClassLoader<KinematicsInterface>";
  }

public:
  std::shared_ptr<kinematics_interface::KinematicsInterface> createSharedInstance(const std::string& plugin_name) const
  {
    return kinematics_loader_->createSharedInstance(plugin_name);
  }

  static const SharedData& instance()
  {
    static SharedData instance;
    return instance;
  }
  static void release()
  {
    SharedData& shared = const_cast<SharedData&>(instance());
    shared.kinematics_loader_.reset();
  }
};

// This class implements the tests
class KinematicsTest : public ::testing::Test
{
protected:
  void operator=(const SharedData& data)
  {
    node = data.node_;
    kinematics_interface_plugin_name = data.kinematics_interface_plugin_name_;
    tip_link = data.tip_link_;
  }

  void SetUp() override
  {
    *this = SharedData::instance();

    // Load KDL inverse dynamics solver plugin
    RCLCPP_INFO_STREAM(node->get_logger(), "Loading " << kinematics_interface_plugin_name);
    kinematics_interface = SharedData::instance().createSharedInstance(kinematics_interface_plugin_name);
    ASSERT_TRUE(bool(kinematics_interface)) << "Failed to load plugin: " << kinematics_interface_plugin_name;
    RCLCPP_INFO_STREAM(node->get_logger(), kinematics_interface_plugin_name << " loaded.");

    // Initialize inverse dynamics solver
    RCLCPP_INFO_STREAM(node->get_logger(), "Initializing " << kinematics_interface_plugin_name);
    ASSERT_TRUE(kinematics_interface->initialize(node->get_node_parameters_interface(), tip_link));
    RCLCPP_INFO_STREAM(node->get_logger(), kinematics_interface_plugin_name << " initialized.");
  }

public:
  rclcpp::Node::SharedPtr node;
  std::string kinematics_interface_plugin_name;
  std::shared_ptr<kinematics_interface::KinematicsInterface> kinematics_interface;
  std::unique_ptr<acg_kinematics::RTKinematicsSolver> robot_kinematics;
  std::string tip_link;
  static constexpr double ABS_EPSILON{ 1e-4 };
};

/**
 * @brief Test the computation of the forward kinematics of the RTKinematicsSolver class
 */
TEST_F(KinematicsTest, TestComputeForwardKinematics)
{
  // Initialize the joint positions
  std::vector<double> joint_positions{ 0.0, 0.0 };

  // Initialize the joint velocities
  std::vector<double> joint_velocities{ 0.0, 0.0 };

  // Initialize the end effector frame
  std::string end_effectors_frame{ "tool_link" };

  // Initialize the desired base frame
  std::string desired_base_frame{ "base_link" };

  // Initialize the expected pose
  geometry_msgs::msg::Pose expected_pose;
  expected_pose.position.x = 0.0;
  expected_pose.position.y = 0.9;
  expected_pose.position.z = 1.2;
  expected_pose.orientation.x = 0.0;
  expected_pose.orientation.y = 0.0;
  expected_pose.orientation.z = 0.0;
  expected_pose.orientation.w = 1.0;

  // Initialize the expected twist
  geometry_msgs::msg::Twist expected_twist;
  expected_twist.linear.x = 0.0;
  expected_twist.linear.y = 0.0;
  expected_twist.linear.z = 0.0;
  expected_twist.angular.x = 0.0;
  expected_twist.angular.y = 0.0;
  expected_twist.angular.z = 0.0;

  // Initialize the task space point
  acg_control_msgs::msg::TaskSpacePoint task_space_point;

  // Initialize the robot kinematics
  robot_kinematics = std::make_unique<acg_kinematics::RTKinematicsSolver>();
  robot_kinematics->initialize(joint_positions.size(), kinematics_interface);

  // Test the zero-order forward kinematics
  robot_kinematics->compute_forward_kinematics(joint_positions, end_effectors_frame, desired_base_frame, task_space_point.pose);

  // Check the pose
  EXPECT_NEAR(task_space_point.pose.position.x, expected_pose.position.x, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.pose.position.y, expected_pose.position.y, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.pose.position.z, expected_pose.position.z, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.pose.orientation.x, expected_pose.orientation.x, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.pose.orientation.y, expected_pose.orientation.y, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.pose.orientation.z, expected_pose.orientation.z, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.pose.orientation.w, expected_pose.orientation.w, ABS_EPSILON);

  // Test the first-order forward kinematics
  robot_kinematics->compute_forward_kinematics(joint_positions, joint_velocities, end_effectors_frame, desired_base_frame, task_space_point);

  // Check the pose
  EXPECT_NEAR(task_space_point.pose.position.x, expected_pose.position.x, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.pose.position.y, expected_pose.position.y, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.pose.position.z, expected_pose.position.z, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.pose.orientation.x, expected_pose.orientation.x, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.pose.orientation.y, expected_pose.orientation.y, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.pose.orientation.z, expected_pose.orientation.z, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.pose.orientation.w, expected_pose.orientation.w, ABS_EPSILON);

  // Check the twist
  EXPECT_NEAR(task_space_point.twist.linear.x, expected_twist.linear.x, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.twist.linear.y, expected_twist.linear.y, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.twist.linear.z, expected_twist.linear.z, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.twist.angular.x, expected_twist.angular.x, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.twist.angular.y, expected_twist.angular.y, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.twist.angular.z, expected_twist.angular.z, ABS_EPSILON);
}

/**
 * @brief Test the RTKinematicsSolver class when it is not initialized
 */
TEST_F(KinematicsTest, TestComputeForwardKinematicsNotInitialized)
{
  // Initialize the task space point
  acg_control_msgs::msg::TaskSpacePoint task_space_point;

  // Initialize the robot kinematics
  robot_kinematics = std::make_unique<acg_kinematics::RTKinematicsSolver>();

  // Compute the forward kinematics
  EXPECT_THROW(robot_kinematics->compute_forward_kinematics(std::vector<double>(), std::vector<double>(), "", "", task_space_point),
               std::runtime_error);
}

/**
 * @brief Test the \c compute_frame_to_frame_transform function of the kinematics library
 */
TEST_F(KinematicsTest, TestComputeFrameToFrameTransform)
{
  // Initialize the joint positions
  std::vector<double> joint_positions{ 0.0, 0.0 };
  std::string start_frame{ "tool_link" };
  std::string end_frame{ "link1" };

  // Expected transform
  Eigen::Quaterniond quaternion(0.707107, -0.707107, 0.0, 0.0);  // (w,x,y,z)
  quaternion.normalize();
  Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
  Eigen::Vector3d translation(0.0, -0.9, -1);

  // Compute the transform
  Eigen::Isometry3d transform;
  acg_kinematics::compute_frame_to_frame_transform(*kinematics_interface, joint_positions, start_frame, end_frame, transform);

  // Check the transform
  EXPECT_NEAR(transform.translation().x(), translation.x(), ABS_EPSILON);
  EXPECT_NEAR(transform.translation().y(), translation.y(), ABS_EPSILON);
  EXPECT_NEAR(transform.translation().z(), translation.z(), ABS_EPSILON);
  for (std::size_t i = 0; i < transform.linear().RowsAtCompileTime; ++i)
  {
    for (std::size_t j = 0; j < transform.linear().ColsAtCompileTime; ++j)
    {
      EXPECT_NEAR(transform.linear()(i, j), rotation_matrix(i, j), ABS_EPSILON);
    }
  }
}

/**
 * @brief Test the \c transform_wrench_frame function of the kinematics library
 */
TEST_F(KinematicsTest, TestChangeWrenchFrame)
{
  // Initialize the joint positions
  std::vector<double> joint_positions{ 0.0, 0.0 };

  // Initialize the desired wrench frame
  std::string desired_wrench_frame{ "tool_link" };

  // Initialize the wrench frame
  std::string wrench_frame{ "link2" };

  // Initialize the wrench
  geometry_msgs::msg::Wrench wrench;
  wrench.force.x = 1.0;
  wrench.force.y = 1.0;
  wrench.force.z = 1.0;
  wrench.torque.x = 2.0;
  wrench.torque.y = 2.0;
  wrench.torque.z = 2.0;

  // The link tool_link has the following orientation in the link2 frame (w,x,y,z):
  Eigen::Quaterniond quaternion(1.0, 0.0, 0.0, 0.0);
  quaternion.normalize();
  Eigen::Vector3d translation(0.0, 0.0, 1);  // origin of the tool_link wrt link2 frame

  Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
  rotation_matrix = rotation_matrix.transpose().eval();

  translation = -rotation_matrix * translation;  // origin of the link2 wrt tool_link frame
  Eigen::Matrix<double, 6, 1> rotated_wrench;
  tf2::fromMsg(wrench, rotated_wrench);

  // Construct the skew-symmetric matrix of the translation vector
  Eigen::Matrix3d skew_symmetric_matrix;
  skew_symmetric_matrix << 0.0, -translation.z(), translation.y(), translation.z(), 0.0, -translation.x(), -translation.y(), translation.x(), 0.0;

  // Compute the transformed torque
  rotated_wrench.head(3) = rotation_matrix * rotated_wrench.head(3);
  rotated_wrench.tail(3) = (skew_symmetric_matrix * rotated_wrench.head(3)) + rotation_matrix * rotated_wrench.tail(3);

  // Initialize the expected wrench
  geometry_msgs::msg::Wrench expected_wrench;
  tf2::toMsg(rotated_wrench, expected_wrench);

  // Change the wrench frame
  acg_kinematics::transform_wrench_frame(*kinematics_interface, joint_positions, desired_wrench_frame, wrench_frame, wrench);

  // Check the wrench
  EXPECT_NEAR(wrench.force.x, expected_wrench.force.x, ABS_EPSILON);
  EXPECT_NEAR(wrench.force.y, expected_wrench.force.y, ABS_EPSILON);
  EXPECT_NEAR(wrench.force.z, expected_wrench.force.z, ABS_EPSILON);
  EXPECT_NEAR(wrench.torque.x, expected_wrench.torque.x, ABS_EPSILON);
  EXPECT_NEAR(wrench.torque.y, expected_wrench.torque.y, ABS_EPSILON);
  EXPECT_NEAR(wrench.torque.z, expected_wrench.torque.z, ABS_EPSILON);
}

/**
 * @brief Test the \c transform_task_space_point_frames function of the kinematics library
 */
TEST_F(KinematicsTest, TestSetTaskSpacePointFrame)
{
  // Initialize the joint positions
  std::vector<double> joint_positions{ 0.0, 0.0 };

  // Initialize the desired motion frame
  std::string desired_motion_frame{ "link1" };

  // Initialize the desired wrench frame
  std::string desired_wrench_frame{ "world" };

  // Initialize the task space point
  std::string motion_frame{ "world" };
  std::string wrench_frame{ "tool_link" };

  // Initialize the task space point
  acg_control_msgs::msg::TaskSpacePoint task_space_point;
  task_space_point.pose.position.x = 1.0;
  task_space_point.pose.position.y = 2.0;
  task_space_point.pose.position.z = 3.0;
  task_space_point.pose.orientation.x = 0.0;
  task_space_point.pose.orientation.y = 0.0;
  task_space_point.pose.orientation.z = 0.0;
  task_space_point.pose.orientation.w = 1.0;
  task_space_point.twist.linear.x = 4.0;
  task_space_point.twist.linear.y = 5.0;
  task_space_point.twist.linear.z = 6.0;
  task_space_point.twist.angular.x = 7.0;
  task_space_point.twist.angular.y = 8.0;
  task_space_point.twist.angular.z = 9.0;
  task_space_point.acceleration.linear.x = 10.0;
  task_space_point.acceleration.linear.y = 11.0;
  task_space_point.acceleration.linear.z = 12.0;
  task_space_point.acceleration.angular.x = 13.0;
  task_space_point.acceleration.angular.y = 14.0;
  task_space_point.acceleration.angular.z = 15.0;
  task_space_point.wrench.force.x = 16.0;
  task_space_point.wrench.force.y = 17.0;
  task_space_point.wrench.force.z = 18.0;
  task_space_point.wrench.torque.x = 19.0;
  task_space_point.wrench.torque.y = 20.0;
  task_space_point.wrench.torque.z = 21.0;
  task_space_point.wrench_derivative.force.x = 22.0;
  task_space_point.wrench_derivative.force.y = 23.0;
  task_space_point.wrench_derivative.force.z = 24.0;
  task_space_point.wrench_derivative.torque.x = 25.0;
  task_space_point.wrench_derivative.torque.y = 26.0;
  task_space_point.wrench_derivative.torque.z = 27.0;
  task_space_point.motion_frame = motion_frame;
  task_space_point.wrench_frame = wrench_frame;

  // Compute the expected pose by applying the desired motion frame transformation
  Eigen::Quaterniond motion_quaternion(0.70710678118, -0.70710678118, 0.0, 0.0);  // (w,x,y,z)
  Eigen::Matrix3d motion_rotation_matrix = motion_quaternion.toRotationMatrix().transpose();
  Eigen::Vector3d motion_displacement(0.0, 0.0, 0.2);
  Eigen::Vector3d position;
  tf2::fromMsg(task_space_point.pose.position, position);
  Eigen::Vector3d transformed_position = motion_rotation_matrix * position - (motion_rotation_matrix * motion_displacement);

  // Initialize the expected point
  acg_control_msgs::msg::TaskSpacePoint expected_point;
  expected_point.pose.position = tf2::toMsg(transformed_position);
  Eigen::Quaterniond pose_quaternion;
  tf2::fromMsg(task_space_point.pose.orientation, pose_quaternion);
  expected_point.pose.orientation = tf2::toMsg(pose_quaternion * motion_quaternion.inverse());

  // Rotate twist and acceleration
  Eigen::Matrix<double, 6, 1> twist, acceleration;
  tf2::fromMsg(task_space_point.twist, twist);
  tf2::fromMsg(task_space_point.acceleration, acceleration);

  // Note that the twist and acceleration must be only rotated, not translated
  Eigen::Matrix<double, 6, 6> rotation_block = Eigen::Matrix<double, 6, 6>::Zero();
  rotation_block.block<3, 3>(0, 0) = rotation_block.block<3, 3>(3, 3) = motion_rotation_matrix;
  twist = rotation_block * twist;
  acceleration = rotation_block * acceleration;
  expected_point.twist = tf2::toMsg(twist);
  tf2::toMsg(acceleration, expected_point.acceleration);

  // Compute the transformed wrench
  Eigen::Matrix<double, 6, 1> wrench;
  tf2::fromMsg(task_space_point.wrench, wrench);

  // The link tool_link has the following orientation in the world frame (w,x,y,z):
  Eigen::Quaterniond wrench_quaternion(1.0, 0.0, 0.0, 0.0);
  wrench_quaternion.normalize();
  Eigen::Matrix3d wrench_rotation_matrix = wrench_quaternion.toRotationMatrix();
  Eigen::Vector3d translation(0.0, 0.9, 1.2);  // origin of the tool_link wrt world frame

  // Construct the skew-symmetric matrix of the translation vector
  Eigen::Matrix3d skew_symmetric_matrix;
  skew_symmetric_matrix << 0.0, -translation.z(), translation.y(), translation.z(), 0.0, -translation.x(), -translation.y(), translation.x(), 0.0;

  // Compute the transformed force and torque
  wrench.head(3) = wrench_rotation_matrix * wrench.head(3);
  wrench.tail(3) = (skew_symmetric_matrix * wrench.head(3)) + (wrench_rotation_matrix * wrench.tail(3));
  tf2::toMsg(wrench, expected_point.wrench);

  // Compute the transformed wrench derivative
  tf2::fromMsg(task_space_point.wrench_derivative, wrench);
  wrench.head(3) = wrench_rotation_matrix * wrench.head(3);
  wrench.tail(3) = (skew_symmetric_matrix * wrench.head(3)) + (wrench_rotation_matrix * wrench.tail(3));
  tf2::toMsg(wrench, expected_point.wrench_derivative);

  acg_kinematics::transform_task_space_point_frames(*kinematics_interface, joint_positions, desired_motion_frame, desired_wrench_frame,
                                                    task_space_point);

  // Check the pose
  EXPECT_NEAR(task_space_point.pose.position.x, expected_point.pose.position.x, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.pose.position.y, expected_point.pose.position.y, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.pose.position.z, expected_point.pose.position.z, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.pose.orientation.x, expected_point.pose.orientation.x, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.pose.orientation.y, expected_point.pose.orientation.y, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.pose.orientation.z, expected_point.pose.orientation.z, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.pose.orientation.w, expected_point.pose.orientation.w, ABS_EPSILON);

  // Check the twist
  EXPECT_NEAR(task_space_point.twist.linear.x, expected_point.twist.linear.x, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.twist.linear.y, expected_point.twist.linear.y, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.twist.linear.z, expected_point.twist.linear.z, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.twist.angular.x, expected_point.twist.angular.x, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.twist.angular.y, expected_point.twist.angular.y, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.twist.angular.z, expected_point.twist.angular.z, ABS_EPSILON);

  // Check the acceleration
  EXPECT_NEAR(task_space_point.acceleration.linear.x, expected_point.acceleration.linear.x, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.acceleration.linear.y, expected_point.acceleration.linear.y, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.acceleration.linear.z, expected_point.acceleration.linear.z, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.acceleration.angular.x, expected_point.acceleration.angular.x, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.acceleration.angular.y, expected_point.acceleration.angular.y, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.acceleration.angular.z, expected_point.acceleration.angular.z, ABS_EPSILON);

  // Check the wrench
  EXPECT_NEAR(task_space_point.wrench.force.x, expected_point.wrench.force.x, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.wrench.force.y, expected_point.wrench.force.y, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.wrench.force.z, expected_point.wrench.force.z, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.wrench.torque.x, expected_point.wrench.torque.x, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.wrench.torque.y, expected_point.wrench.torque.y, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.wrench.torque.z, expected_point.wrench.torque.z, ABS_EPSILON);

  // Check the wrench derivative
  EXPECT_NEAR(task_space_point.wrench_derivative.force.x, expected_point.wrench_derivative.force.x, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.wrench_derivative.force.y, expected_point.wrench_derivative.force.y, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.wrench_derivative.force.z, expected_point.wrench_derivative.force.z, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.wrench_derivative.torque.x, expected_point.wrench_derivative.torque.x, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.wrench_derivative.torque.y, expected_point.wrench_derivative.torque.y, ABS_EPSILON);
  EXPECT_NEAR(task_space_point.wrench_derivative.torque.z, expected_point.wrench_derivative.torque.z, ABS_EPSILON);

  // Check the motion frame
  EXPECT_EQ(task_space_point.motion_frame, desired_motion_frame);

  // Check the wrench frame
  EXPECT_EQ(task_space_point.wrench_frame, desired_wrench_frame);
}

// ****** Definitions of the test parameters and test cases for compute_pose_error ******

struct ComputePoseErrorTestParams
{
  geometry_msgs::msg::Pose desired_pose;
  geometry_msgs::msg::Pose current_pose;
  Eigen::Matrix<double, 6, 1> expected_error;
};

class ComputePoseErrorTestSuite : public ::testing::TestWithParam<ComputePoseErrorTestParams>
{};

/**
 * @brief Test the \c compute_pose_error function of the kinematics utilities library
 */
TEST_P(ComputePoseErrorTestSuite, ComputePoseError)
{
  static constexpr double ABS_EPSILON{ 1e-6 };
  const ComputePoseErrorTestParams param = GetParam();
  Eigen::Matrix<double, 6, 1> error;
  acg_kinematics::compute_pose_error(param.desired_pose, param.current_pose, error);
  for (Eigen::Index i = 0; i < param.expected_error.size(); ++i)
  {
    EXPECT_NEAR(error(i), param.expected_error(i), ABS_EPSILON) << "Error at index " << i << ": " << error(i) << " != " << param.expected_error(i);
  }
}

static ComputePoseErrorTestParams createEdgeCaseEqualPoses()
{
  ComputePoseErrorTestParams params;
  params.desired_pose.position.x = 0.042641;
  params.desired_pose.position.y = 0.618020;
  params.desired_pose.position.z = 0.595680;
  params.desired_pose.orientation.x = -0.728750;
  params.desired_pose.orientation.y = 0.129740;
  params.desired_pose.orientation.z = -0.190550;
  params.desired_pose.orientation.w = 0.644808;
  params.current_pose = params.desired_pose;
  params.expected_error.setZero();
  return params;
}

static ComputePoseErrorTestParams create180degZError()
{
  ComputePoseErrorTestParams params;
  params.current_pose.position.x = 1;
  params.current_pose.position.y = 2;
  params.current_pose.position.z = 3;
  params.current_pose.orientation.x = 0.0;
  params.current_pose.orientation.y = 0.0;
  params.current_pose.orientation.z = 0.0;
  params.current_pose.orientation.w = 1.0;

  params.desired_pose.position.x = 1;
  params.desired_pose.position.y = 2;
  params.desired_pose.position.z = 3;
  params.desired_pose.orientation.x = 0.0;
  params.desired_pose.orientation.y = 0.0;
  params.desired_pose.orientation.z = 1.0;
  params.desired_pose.orientation.w = 0.0;

  params.expected_error << 0.0, 0.0, 0.0, 0.0, 0.0, M_PI;
  return params;
}

INSTANTIATE_TEST_SUITE_P(KinematicsUtilitiesTestComputePoseError, ComputePoseErrorTestSuite,
                         ::testing::Values(createEdgeCaseEqualPoses(), create180degZError()));

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  SharedData::release();
  return result;
}
