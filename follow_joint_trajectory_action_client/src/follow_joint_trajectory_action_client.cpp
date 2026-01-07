/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   follow_joint_trajectory_action_client.cpp
 * Author:  Adamo Zito, Francesco D'Onofrio
 * Org.:    UNISA
 * Date:    July 17, 2025
 *
 * This node instantiates an action client in order to communicate
 * with an action server using the FollowJointTrajectory action.
 * This way, the trajectory execution can be monitored and data logged
 * to bag file. The node opens a bag file with a joint trajectory of
 * type xxx_control_msgs/JointTrajectory, reads the trajectory
 * contained in it and sends it to the controller. Another bag file is
 * written containing the actual, desired and error trajectories.
 *
 * -------------------------------------------------------------------
 */

// Standard libraries
#include <filesystem>
#include <rcutils/time.h>

// ROS
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <builtin_interfaces/msg/time.hpp>

// Rosbag
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

// Messages
#include <xxx_control_msgs/msg/joint_trajectory.hpp>
#include <xxx_control_msgs/action/follow_joint_trajectory.hpp>

namespace follow_joint_trajectory_action_client_cpp
{
class FollowJointTrajectoryActionClient : public rclcpp::Node
{
  using FollowJointTrajectoryGoalHandle = typename rclcpp_action::ClientGoalHandle<xxx_control_msgs::action::FollowJointTrajectory>;

public:
  explicit FollowJointTrajectoryActionClient(const rclcpp::NodeOptions& options) : Node("follow_joint_trajectory_action_client", options)
  {
    std::string action_name, input_trajectory_filename;

    // Declare node's parameters
    this->declare_parameter<std::string>("action_name");
    this->declare_parameter<std::string>("input_trajectory_filename");
    this->declare_parameter<int>("fraction_feedback_messages_to_save", 25);
    this->declare_parameter<std::string>("output_path", "");

    // Retrieve the node's parameters
    this->get_parameter("action_name", action_name);
    this->get_parameter("input_trajectory_filename", input_trajectory_filename);
    if (input_trajectory_filename.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "'input_trajectory_filename' is empty.");
      rclcpp::shutdown();
      return;
    }
    this->get_parameter("fraction_feedback_messages_to_save", fraction_feedback_messages_to_save_);
    if (fraction_feedback_messages_to_save_ <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "'fraction_feedback_messages_to_save' should be greater than zero. Please check the parameter configuration.");
      rclcpp::shutdown();
      return;
    }

    // Finding the share directory of this package
    std::filesystem::path package_share_directory(ament_index_cpp::get_package_share_directory("follow_joint_trajectory_action_client"));

    // Defining the path to the bag file containing the input trajectory to use
    std::filesystem::path input_path = package_share_directory / "trajectories" / input_trajectory_filename;

    // Defining the path to the directory in which to store the output of the action client
    output_path_ = this->get_parameter("output_path").as_string();

    // Initialize the variables that will contain trajectories
    input_trajectory_ = std::make_shared<xxx_control_msgs::msg::JointTrajectory>();
    output_actual_trajectory_ = std::make_shared<xxx_control_msgs::msg::JointTrajectory>();
    output_desired_trajectory_ = std::make_shared<xxx_control_msgs::msg::JointTrajectory>();
    output_error_trajectory_ = std::make_shared<xxx_control_msgs::msg::JointTrajectory>();

    // Read the input trajectory from a bag file
    rclcpp::Serialization<xxx_control_msgs::msg::JointTrajectory> serialization;
    rosbag2_cpp::Reader reader;

    reader.open(input_path);
    bool found_trajectory = false;
    while (reader.has_next())
    {
      rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();
      if (msg->topic_name != "/joint_trajectory")
      {
        continue;
      }
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      serialization.deserialize_message(&serialized_msg, input_trajectory_.get());
      found_trajectory = true;
      break;
    }
    reader.close();
    if (!found_trajectory || input_trajectory_->points.size() == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "No valid joint trajectory found in input bag file: %s.", input_path.c_str());
      rclcpp::shutdown();
      return;
    }

    output_actual_trajectory_->header = input_trajectory_->header;
    output_desired_trajectory_->header = input_trajectory_->header;
    output_error_trajectory_->header = input_trajectory_->header;

    output_actual_trajectory_->joint_names = input_trajectory_->joint_names;
    output_desired_trajectory_->joint_names = input_trajectory_->joint_names;
    output_error_trajectory_->joint_names = input_trajectory_->joint_names;

    // Creation of an action client
    client_ptr_ = rclcpp_action::create_client<xxx_control_msgs::action::FollowJointTrajectory>(this, action_name + "/follow_joint_trajectory");

    // Send the first trajectory point and afterwards send the whole trajectory
    if (send_first_trajectory_point())
    {
      // Waiting some time before to be sure that the robot is stable before executing the whole trajectory
      rclcpp::sleep_for(std::chrono::seconds(3));

      send_trajectory();
    }
  }

  bool send_first_trajectory_point()
  {
    // Waiting for the action server
    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(30)))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting.");
      rclcpp::shutdown();
      return false;
    }

    // Create a goal message containing only the first trajectory point
    xxx_control_msgs::action::FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory.points.push_back(input_trajectory_.get()->points[0]);
    goal_msg.trajectory.joint_names = input_trajectory_.get()->joint_names;
    goal_msg.trajectory.header = input_trajectory_.get()->header;

    // Sending the goal to the action server
    std::shared_future<FollowJointTrajectoryGoalHandle::SharedPtr> future_goal_handle = client_ptr_->async_send_goal(goal_msg);

    // Waiting at most 20 seconds for the goal to be received
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle, std::chrono::seconds(20)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to send goal request.");
      rclcpp::shutdown();
      return false;
    }

    // Verify if the goal has been accepted
    FollowJointTrajectoryGoalHandle::SharedPtr goal_handle = future_goal_handle.get();
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
      rclcpp::shutdown();
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result...");

    // Get the result future
    std::shared_future<FollowJointTrajectoryGoalHandle::WrappedResult> result_future = client_ptr_->async_get_result(goal_handle);

    // Wait for the result synchronously with a 20-second timeout
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future, std::chrono::seconds(20)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get result within the 20-second timeout.");
      rclcpp::shutdown();
      return false;
    }

    if (result_future.get().code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_ERROR(this->get_logger(), "The robot was not able to reach the set point.");
      rclcpp::shutdown();
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Set point reached.");
    return true;
  }

  void send_trajectory()
  {
    // Wait for the action server
    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(30)))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting.");
      rclcpp::shutdown();
    }

    // Create a goal message containing the whole trajectory
    xxx_control_msgs::action::FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory.points = input_trajectory_.get()->points;
    goal_msg.trajectory.joint_names = input_trajectory_.get()->joint_names;
    goal_msg.trajectory.header = input_trajectory_.get()->header;

    // Sending the goal to the action server
    rclcpp_action::Client<xxx_control_msgs::action::FollowJointTrajectory>::SendGoalOptions send_goal_options;
    send_goal_options.goal_response_callback = std::bind(&FollowJointTrajectoryActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&FollowJointTrajectoryActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&FollowJointTrajectoryActionClient::result_callback, this, std::placeholders::_1);
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<xxx_control_msgs::action::FollowJointTrajectory>::SharedPtr client_ptr_;
  xxx_control_msgs::msg::JointTrajectory::SharedPtr input_trajectory_, output_actual_trajectory_, output_desired_trajectory_,
      output_error_trajectory_;
  int fraction_feedback_messages_to_save_{ 0 }, number_feedback_messages_received_{ 0 };
  builtin_interfaces::msg::Time controller_first_time_;
  std::filesystem::path output_path_;

  void goal_response_callback(const FollowJointTrajectoryGoalHandle::SharedPtr& goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result...");
    }
  }

  void feedback_callback(rclcpp_action::ClientGoalHandle<xxx_control_msgs::action::FollowJointTrajectory>::SharedPtr,
                         const std::shared_ptr<const xxx_control_msgs::action::FollowJointTrajectory::Feedback> feedback)
  {
    // Set controller_first_time_ on the first received feedback message
    if (number_feedback_messages_received_ == 0)
    {
      controller_first_time_ = feedback->header.stamp;
    }
    number_feedback_messages_received_++;

    // compute time_from_start using rclcpp time/duration helpers
    rclcpp::Time t_now(feedback->header.stamp);
    rclcpp::Time t_start(controller_first_time_);
    rclcpp::Duration dt = t_now - t_start;
    const int64_t ns = dt.nanoseconds();
    builtin_interfaces::msg::Duration time_from_start;
    time_from_start.sec = static_cast<int32_t>(ns / 1000000000LL);
    time_from_start.nanosec = static_cast<uint32_t>(ns % 1000000000LL);

    // Since the feedback cannot be modified, create copies and preserve the computed time_from_start
    if (number_feedback_messages_received_ % fraction_feedback_messages_to_save_ == 0)
    {
      auto actual_point = feedback->actual;
      actual_point.time_from_start = time_from_start;
      output_actual_trajectory_->points.push_back(actual_point);

      auto desired_point = feedback->desired;
      desired_point.time_from_start = time_from_start;
      output_desired_trajectory_->points.push_back(desired_point);

      auto error_point = feedback->error;
      error_point.time_from_start = time_from_start;
      output_error_trajectory_->points.push_back(error_point);
    }
  }

  void result_callback(const FollowJointTrajectoryGoalHandle::WrappedResult& result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
      {
        // Writing trajectory logs to bag files
        // Obtaining the current time
        std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm local_tm = *std::localtime(&now);
        std::ostringstream now_str;
        now_str << std::put_time(&local_tm, "%Y-%m-%d-%H-%M-%S");
        std::string current_time = now_str.str();

        // Serialize the messages
        rclcpp::SerializedMessage serialized_output_actual_trajectory, serialized_output_desired_trajectory, serialized_output_error_trajectory;
        rclcpp::Serialization<xxx_control_msgs::msg::JointTrajectory> serialization;
        serialization.serialize_message(output_actual_trajectory_.get(), &serialized_output_actual_trajectory);
        serialization.serialize_message(output_desired_trajectory_.get(), &serialized_output_desired_trajectory);
        serialization.serialize_message(output_error_trajectory_.get(), &serialized_output_error_trajectory);

        // Initialize and open the bag file
        rosbag2_cpp::Writer writer;
        writer.open((output_path_ / ("follow_joint_trajectory_result_" + current_time)).string());

        rosbag2_storage::SerializedBagMessageSharedPtr bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        rcutils_ret_t ret = rcutils_system_time_now(&bag_message->time_stamp);
        if (ret != RCUTILS_RET_OK)
        {
          RCLCPP_ERROR(this->get_logger(), "Failed to get system time: %s", rcutils_get_error_string().str);
          rcutils_reset_error();
          writer.close();
          return;
        }

        // Topic creation on the bag file
        rosbag2_storage::TopicMetadata tm;
        tm.name = "/follow_joint_trajectory_result";
        tm.type = "xxx_control_msgs/msg/JointTrajectory";
        tm.serialization_format = "cdr";
        writer.create_topic(tm);

        // Writing on the bag file
        bag_message->topic_name = tm.name;
        bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(&serialized_output_actual_trajectory.get_rcl_serialized_message(),
                                                                              [](rcutils_uint8_array_t* /* data */) {});
        writer.write(bag_message);
        bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(&serialized_output_desired_trajectory.get_rcl_serialized_message(),
                                                                              [](rcutils_uint8_array_t* /* data */) {});
        writer.write(bag_message);
        bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(&serialized_output_error_trajectory.get_rcl_serialized_message(),
                                                                              [](rcutils_uint8_array_t* /* data */) {});
        writer.write(bag_message);

        writer.close();

        RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
        break;
      }
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted.");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled.");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
        return;
    }

    rclcpp::shutdown();
  }
};  // class FollowJointTrajectoryActionClient

}  // namespace follow_joint_trajectory_action_client_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(follow_joint_trajectory_action_client_cpp::FollowJointTrajectoryActionClient)
