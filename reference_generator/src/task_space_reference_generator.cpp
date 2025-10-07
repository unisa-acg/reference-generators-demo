/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   task_space_reference_generator.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Nov 18, 2024
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <rclcpp/logging.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // for tf2::fromMsg

#include <acg_common_libraries/diagnostics.hpp>
#include <acg_common_libraries/interpolation.hpp>
#include <acg_common_libraries/message_utilities.hpp>

#include "reference_generator/task_space_reference_generator.hpp"
namespace task_space_reference_generator
{

TaskSpaceReferenceGenerator::TaskSpaceReferenceGenerator() : ReferenceGenerator(trajectory_info_) {}

controller_interface::CallbackReturn TaskSpaceReferenceGenerator::on_init()
{
  // Call the base class implementation
  controller_interface::CallbackReturn ret = ReferenceGenerator::on_init();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  try
  {
    // Create the parameter listener and get the parameters
    parameter_handler_ = std::make_shared<task_space_reference_generator::ParamListener>(get_node());
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "Exception thrown during init stage with message: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // Setting the command interface names override struct based on the configuration file
  acg_hardware_interface_facade::CommandInterfaceNamesOverrideConfig command_interface_names_override;
  command_interface_names_override.task_space_pose_interface_names =
      parameter_handler_->get_params().command_interfaces_names_override.task_space_pose;
  command_interface_names_override.task_space_twist_interface_names =
      parameter_handler_->get_params().command_interfaces_names_override.task_space_velocity;
  command_interface_names_override.task_space_acceleration_interface_names =
      parameter_handler_->get_params().command_interfaces_names_override.task_space_acceleration;
  command_interface_names_override.task_space_wrench_interface_names =
      parameter_handler_->get_params().command_interfaces_names_override.task_space_wrench;
  command_interface_names_override.task_space_wrench_derivative_interface_names =
      parameter_handler_->get_params().command_interfaces_names_override.task_space_wrench_derivative;

  // Configure the command interfaces
  command_writer_.configure_interfaces(parameter_handler_->get_params().joints, std::vector<std::string>(),
                                       parameter_handler_->get_params().task_space_command_interfaces, std::string(),
                                       parameter_handler_->get_params().task_space_command_controller, command_interface_names_override);

  // Initialize robot kinematics
  robot_kinematics_.initialize(parameter_handler_->get_params().joints.size(), ReferenceGenerator::kinematics_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TaskSpaceReferenceGenerator::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  // Call the base class implementation
  controller_interface::CallbackReturn ret = ReferenceGenerator::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  current_traj_point_.point.motion_frame.reserve(ReferenceGenerator::STRING_INITIAL_CAPACITY_);
  current_traj_point_.point.wrench_frame.reserve(ReferenceGenerator::STRING_INITIAL_CAPACITY_);
  next_traj_point_.point.motion_frame.reserve(ReferenceGenerator::STRING_INITIAL_CAPACITY_);
  next_traj_point_.point.wrench_frame.reserve(ReferenceGenerator::STRING_INITIAL_CAPACITY_);

  // Initialize feedback
  feedback_ = std::make_shared<TaskSpaceTrajFeedback>();
  feedback_->header.frame_id.reserve(ReferenceGenerator::STRING_INITIAL_CAPACITY_);
  feedback_->desired.point.motion_frame.reserve(ReferenceGenerator::STRING_INITIAL_CAPACITY_);
  feedback_->desired.point.wrench_frame.reserve(ReferenceGenerator::STRING_INITIAL_CAPACITY_);
  feedback_->actual.point.motion_frame.reserve(ReferenceGenerator::STRING_INITIAL_CAPACITY_);
  feedback_->actual.point.wrench_frame.reserve(ReferenceGenerator::STRING_INITIAL_CAPACITY_);
  feedback_->error.point.motion_frame.reserve(ReferenceGenerator::STRING_INITIAL_CAPACITY_);
  feedback_->error.point.wrench_frame.reserve(ReferenceGenerator::STRING_INITIAL_CAPACITY_);
  robot_actual_task_space_state_.motion_frame.reserve(ReferenceGenerator::STRING_INITIAL_CAPACITY_);
  robot_actual_task_space_state_.wrench_frame.reserve(ReferenceGenerator::STRING_INITIAL_CAPACITY_);

  // Configure the action server for the task space reference
  task_space_trajectory_action_server_ = rclcpp_action::create_server<acg_control_msgs::action::FollowTaskSpaceTrajectory>(
      get_node(), "~/follow_task_space_trajectory",
      std::bind(&TaskSpaceReferenceGenerator::handle_goal_, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TaskSpaceReferenceGenerator::handle_cancel_, this, std::placeholders::_1),
      std::bind(&TaskSpaceReferenceGenerator::handle_accepted_, this, std::placeholders::_1));

  // Set up QoS settings
  rclcpp::QoS qos{ rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)) };
  qos.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  // Configure the subscriber for the task space reference
  task_space_reference_subscriber_ = get_node()->create_subscription<acg_control_msgs::msg::TaskSpacePoint>(
      "~/reference", qos, std::bind(&TaskSpaceReferenceGenerator::task_space_reference_callback_, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TaskSpaceReferenceGenerator::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  // Call the base class implementation
  controller_interface::CallbackReturn ret = ReferenceGenerator::on_activate(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Assigning the loaned command interfaces to the internal command writer
  command_writer_.assign_loaned_command_interfaces(ControllerInterfaceBase::command_interfaces_);

  // Clear the task space reference, so that the controller does not use the previous reference
  acg_message_utilities::clear(task_space_reference_);
  task_space_reference_.motion_frame = ReferenceGenerator::task_space_reference_frame_;
  task_space_reference_.wrench_frame = ReferenceGenerator::wrench_reference_frame_;

  // Initialize the task space reference by computing the forward kinematics
  if (command_writer_.has_task_space_pose_interface())
  {
    robot_kinematics_.compute_forward_kinematics(robot_joint_state_.positions, ReferenceGenerator::tip_link_,
                                                 ReferenceGenerator::task_space_reference_frame_, task_space_reference_.pose);
  }

  // Note that the velocities and acceleration are set to zero because the robot should not move when the controller is activated
  if (command_writer_.has_task_space_twist_interface())
  {
    task_space_reference_.twist = geometry_msgs::msg::Twist();
  }
  if (command_writer_.has_task_space_acceleration_interface())
  {
    task_space_reference_.acceleration = geometry_msgs::msg::Accel();
  }

  // Wrench and wrench derivative are set to zero because the robot should not exert any force when the controller is activated
  if (command_writer_.has_task_space_wrench_interface())
  {
    task_space_reference_.wrench = geometry_msgs::msg::Wrench();
  }
  if (command_writer_.has_task_space_wrench_derivative_interface())
  {
    task_space_reference_.wrench_derivative = geometry_msgs::msg::Wrench();
  }

  // Write the task space reference in the task space real-time buffer
  task_space_reference_buffer_.writeFromNonRT(std::make_shared<acg_control_msgs::msg::TaskSpacePoint>(task_space_reference_));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration TaskSpaceReferenceGenerator::command_interface_configuration() const
{
  // Configure the command interface for the task space reference generator
  std::string task_space_command_controller_name = parameter_handler_->get_params().task_space_command_controller;

  if (task_space_command_controller_name.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No command controller specified in the configuration file.");
    return { controller_interface::interface_configuration_type::NONE, {} };
  }

  return { controller_interface::interface_configuration_type::INDIVIDUAL, command_writer_.available_interfaces() };
}

void TaskSpaceReferenceGenerator::abort_trajectory(const std::string& error_string)
{
  if (trajectory_info_.goal_handle_)
  {
    std::shared_ptr<acg_control_msgs::action::FollowTaskSpaceTrajectory_Result> result{
      std::make_shared<acg_control_msgs::action::FollowTaskSpaceTrajectory::Result>()
    };
    result->error_code = acg_control_msgs::action::FollowTaskSpaceTrajectory::Result::TRAJECTORY_ABORTED;
    result->error_string = error_string;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<acg_control_msgs::action::FollowTaskSpaceTrajectory>> goal_handle = trajectory_info_.goal_handle_;
    goal_handle->abort(result);
    goal_handle.reset();
    trajectory_info_ = TaskSpaceTrajectoryInfo();
  }
  trigger_trajectory_cancelled_event();
}

void TaskSpaceReferenceGenerator::read_references_from_non_rt()
{
  task_space_reference_ = **task_space_reference_buffer_.readFromRT();
}

void TaskSpaceReferenceGenerator::write_references_to_command_interfaces()
{
  // Write the task space reference to the command interfaces
  command_writer_.write_to_command_interfaces(task_space_reference_);
}

void TaskSpaceReferenceGenerator::calculate_and_publish_feedback(const rclcpp::Time& time)
{
  feedback_->header.stamp = time;

  // Update the desired trajectory feedback
  if (trajectory_info_.first_point_executed_)
  {
    feedback_->desired.point = task_space_reference_;
  }
  else
  {
    // The trajectory has not started yet, so the reference is set to NaN
    acg_message_utilities::clear(feedback_->desired.point);
  }

  state_reader_.read_state_interfaces(robot_joint_state_);
  acg_message_utilities::clear(robot_actual_task_space_state_);
  robot_actual_task_space_state_.motion_frame = ReferenceGenerator::task_space_reference_frame_;
  robot_actual_task_space_state_.wrench_frame = ReferenceGenerator::wrench_reference_frame_;
  robot_kinematics_.compute_forward_kinematics(robot_joint_state_.positions, robot_joint_state_.velocities, ReferenceGenerator::tip_link_,
                                               ReferenceGenerator::task_space_reference_frame_, robot_actual_task_space_state_);

  feedback_->actual.point = robot_actual_task_space_state_;

  feedback_->error.point = compute_task_space_error_(feedback_);
  trajectory_info_.goal_handle_->publish_feedback(feedback_);
}

std::size_t TaskSpaceReferenceGenerator::get_next_trajectory_index()
{
  const acg_control_msgs::msg::TaskSpaceTrajectory& task_space_trajectory = trajectory_info_.goal_handle_->get_goal()->task_space_trajectory;
  // Getting the index of the last trajectory waypoint
  std::size_t trajectory_index = trajectory_info_.trajectory_index_;
  // Getting the current time of the trajectory
  const rclcpp::Time task_space_trajectory_time = trajectory_info_.current_time_;

  // The next waypoint to command is the last one whose timestamp is less than the controller's internal time
  rclcpp::Duration time_from_start(task_space_trajectory.points[trajectory_index].time_from_start);
  while (trajectory_index < task_space_trajectory.points.size() && time_from_start.seconds() <= task_space_trajectory_time.seconds())
  {
    trajectory_index++;
    time_from_start = rclcpp::Duration(task_space_trajectory.points[trajectory_index].time_from_start);
  }
  return trajectory_index;
}

bool TaskSpaceReferenceGenerator::update_next_reference_from_trajectory()
{
  const acg_control_msgs::msg::TaskSpaceTrajectory& task_space_trajectory = trajectory_info_.goal_handle_->get_goal()->task_space_trajectory;

  current_traj_point_ = task_space_trajectory.points[trajectory_info_.trajectory_index_];

  // Get the current joint positions to apply the correct transformations
  state_reader_.read_state_interfaces(robot_joint_state_);

  // Update the wrench and motion frame of the next point, if they are different from the desired ones
  try
  {
    acg_kinematics::transform_task_space_point_frames(*kinematics_, robot_joint_state_.positions, ReferenceGenerator::task_space_reference_frame_,
                                                      ReferenceGenerator::wrench_reference_frame_, current_traj_point_.point);
  }
  catch (const std::runtime_error& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to transform task space point frames: %s", e.what());
    return false;
  }

  if (trajectory_info_.trajectory_index_ == task_space_trajectory.points.size() - 1)
  {
    task_space_reference_ = current_traj_point_.point;
  }
  else
  {
    next_traj_point_ = task_space_trajectory.points[trajectory_info_.trajectory_index_ + 1];

    // Update the wrench and motion frame of the next point, if they are different from the desired ones
    try
    {
      acg_kinematics::transform_task_space_point_frames(*kinematics_, robot_joint_state_.positions, ReferenceGenerator::task_space_reference_frame_,
                                                        ReferenceGenerator::wrench_reference_frame_, next_traj_point_.point);
    }
    catch (const std::runtime_error& e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to transform task space point frames: %s", e.what());
      return false;
    }

    // Interpolate the task space reference between the current and next trajectory points
    try
    {
      acg_interpolation::linearly_interpolate(current_traj_point_, next_traj_point_, trajectory_info_.current_time_.seconds(), task_space_reference_);
    }
    catch (const std::runtime_error& e)
    {
      return false;
    }
  }

  return true;
}

void TaskSpaceReferenceGenerator::handle_trajectory_completed()
{
  const acg_control_msgs::msg::TaskSpaceTrajectory& task_space_trajectory = trajectory_info_.goal_handle_->get_goal()->task_space_trajectory;
  if (trajectory_info_.trajectory_index_ == task_space_trajectory.points.size() - 1)
  {
    std::shared_ptr<acg_control_msgs::action::FollowTaskSpaceTrajectory_Result> result =
        std::make_shared<acg_control_msgs::action::FollowTaskSpaceTrajectory::Result>();
    result->error_code = acg_control_msgs::action::FollowTaskSpaceTrajectory::Result::SUCCESSFUL;
    trajectory_info_.goal_handle_->succeed(result);

    trajectory_info_.goal_handle_.reset();
    // TODO: Need a lock-free write here. This is a known issue of real-time buffer.
    // See https://github.com/ros-controls/ros2_controllers/issues/168 and https://github.com/ros-controls/realtime_tools/issues/14
    trigger_trajectory_completed_event();
    trajectory_info_ = TaskSpaceTrajectoryInfo();
    trajectory_info_buffer_.writeFromNonRT(trajectory_info_);
  }
}

void TaskSpaceReferenceGenerator::publish_reference_pose(const rclcpp::Time& time)
{
  // Publish the reference so that it can be visualized in RViz.
  if (is_publish_desired_)
  {
    periodic_reference_publisher_->publish(
        acg_message_utilities::build_pose_stamped_msg(task_space_reference_.pose, ReferenceGenerator::task_space_reference_frame_, time), time);
  }
}

void TaskSpaceReferenceGenerator::read_trajectory_info_from_non_rt()
{
  // Get the trajectory info from the real-time buffer
  trajectory_info_ = *trajectory_info_buffer_.readFromRT();
}

rclcpp_action::GoalResponse TaskSpaceReferenceGenerator::handle_goal_(
    const rclcpp_action::GoalUUID& /* uuid */, std::shared_ptr<const acg_control_msgs::action::FollowTaskSpaceTrajectory::Goal> goal)
{
  if (goal->task_space_trajectory.points.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "The received task space trajectory has been rejected because the number of points in the trajectory is %ld",
                 goal->task_space_trajectory.points.size());
    return rclcpp_action::GoalResponse::REJECT;
  }

  const rclcpp::Time trajectory_start_time{ rclcpp::Time(goal->task_space_trajectory.header.stamp) };

  // If the starting time is set to 0.0, it means the controller should begin executing the trajectory immediately; this is a convention.
  // Otherwise we check if the task space trajectory ends before the current time,
  // in which case it can be ignored. In the case described above (starting time = 0.0), this check is not done because
  // trajectory_end_time is always less than the current time.
  if (trajectory_start_time.seconds() != 0.0)
  {
    const rclcpp::Time trajectory_end_time{ trajectory_start_time + rclcpp::Duration(goal->task_space_trajectory.points.back().time_from_start) };
    const rclcpp::Time curr_time{ get_node()->now() };
    if (trajectory_end_time < curr_time)
    {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Received task space trajectory with non-zero start time (%f) that ends in the past (%f). Current time is: %f",
                   trajectory_start_time.seconds(), trajectory_end_time.seconds(), curr_time.seconds());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  bool no_motion_frame{ false };
  bool no_wrench_frame{ false };
  for (std::size_t i = 0; i < goal->task_space_trajectory.points.size(); i++)
  {
    if (!verify_frame_names_(goal->task_space_trajectory.points[i].point.motion_frame, goal->task_space_trajectory.points[i].point.wrench_frame))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "The task space trajectory has been rejected because the frame names are not valid");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->task_space_trajectory.points[i].point.motion_frame.empty())
    {
      no_motion_frame = true;
    }

    if (goal->task_space_trajectory.points[i].point.wrench_frame.empty())
    {
      no_wrench_frame = true;
    }

    if ((i > 0) && (rclcpp::Duration(goal->task_space_trajectory.points[i].time_from_start) <=
                    rclcpp::Duration(goal->task_space_trajectory.points[i - 1].time_from_start)))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Time between points %zu and %zu is not strictly increasing, it is %f and %f respectively", i - 1, i,
                   rclcpp::Duration(goal->task_space_trajectory.points[i - 1].time_from_start).seconds(),
                   rclcpp::Duration(goal->task_space_trajectory.points[i].time_from_start).seconds());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  if (no_motion_frame)
  {
    RCLCPP_WARN(get_node()->get_logger(),
                "No motion frame for at least one point of the incoming task space trajectory. %s frame is used as default for those points.",
                ReferenceGenerator::task_space_reference_frame_.c_str());
  }

  if (no_wrench_frame)
  {
    RCLCPP_WARN(get_node()->get_logger(),
                "No wrench frame for at least one point of the incoming task space trajectory. %s frame is used as default for those points.",
                ReferenceGenerator::wrench_reference_frame_.c_str());
  }

  RCLCPP_INFO(get_node()->get_logger(), "Received a new task space trajectory");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TaskSpaceReferenceGenerator::handle_cancel_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<acg_control_msgs::action::FollowTaskSpaceTrajectory>> /* goal_handle */)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received request to cancel task space trajectory goal");

  trigger_trajectory_cancelled_event();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TaskSpaceReferenceGenerator::handle_accepted_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<acg_control_msgs::action::FollowTaskSpaceTrajectory>> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received request to execute task space trajectory goal");
  // Create a new trajectory info object and store the goal handle
  TaskSpaceTrajectoryInfo trajectory_info;
  trajectory_info.goal_handle_ = goal_handle;

  // TODO: Need a lock-free write here. This is a known issue of real-time buffer.
  // See https://github.com/ros-controls/ros2_controllers/issues/168 and https://github.com/ros-controls/realtime_tools/issues/14
  trajectory_info_buffer_.writeFromNonRT(trajectory_info);

  trigger_trajectory_accepted_event();
}

void TaskSpaceReferenceGenerator::task_space_reference_callback_(const std::shared_ptr<acg_control_msgs::msg::TaskSpacePoint> msg)
{
  RCLCPP_INFO(get_node()->get_logger(), "New task space reference received");

  if (!verify_frame_names_(msg->motion_frame, msg->wrench_frame))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "The task space reference has been rejected because the frame names are not valid");
    return;
  }

  if (msg->wrench_frame.empty())
  {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "No wrench reference frame specified in the task space reference message. Assuming the wrench is expressed in the desired wrench frame: %s",
        ReferenceGenerator::wrench_reference_frame_.c_str());
    msg->wrench_frame = ReferenceGenerator::wrench_reference_frame_;
  }

  if (msg->motion_frame.empty())
  {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "No motion reference frame specified in the task space reference message. Assuming the motion is expressed in the desired motion frame: %s",
        ReferenceGenerator::task_space_reference_frame_.c_str());
    msg->motion_frame = ReferenceGenerator::task_space_reference_frame_;
  }

  // convert the task space point just received from the reference motion and wrench frame (the ones contained in the msg) to the desired ones (the
  // ones set in the configuration file of the reference_generator)
  state_reader_.read_state_interfaces(robot_joint_state_);
  acg_kinematics::transform_task_space_point_frames(*kinematics_, robot_joint_state_.positions, ReferenceGenerator::task_space_reference_frame_,
                                                    ReferenceGenerator::wrench_reference_frame_, *msg);

  task_space_reference_buffer_.writeFromNonRT(msg);

  trigger_publish_on_topic_event();
}

acg_control_msgs::msg::TaskSpacePoint
TaskSpaceReferenceGenerator::compute_task_space_error_(const std::shared_ptr<acg_control_msgs::action::FollowTaskSpaceTrajectory::Feedback>& feedback)
{
  acg_control_msgs::msg::TaskSpacePoint error;
  error.pose.position.x = feedback->actual.point.pose.position.x - feedback->desired.point.pose.position.x;
  error.pose.position.y = feedback->actual.point.pose.position.y - feedback->desired.point.pose.position.y;
  error.pose.position.z = feedback->actual.point.pose.position.z - feedback->desired.point.pose.position.z;

  // Given two quaternions q1 and q2, the error quaternion is defined as q2 * q1^-1
  // Define the actual and desired quaternions
  tf2::Quaternion actual_quat, desired_quat, error_quat;
  tf2::fromMsg(feedback->actual.point.pose.orientation, actual_quat);
  tf2::fromMsg(feedback->desired.point.pose.orientation, desired_quat);

  // Compute the quaternion error.
  error_quat = desired_quat * actual_quat.inverse();
  error_quat.normalize();
  error.pose.orientation = tf2::toMsg(error_quat);

  // Compute the twist error
  error.twist.linear.x = feedback->actual.point.twist.linear.x - feedback->desired.point.twist.linear.x;
  error.twist.linear.y = feedback->actual.point.twist.linear.y - feedback->desired.point.twist.linear.y;
  error.twist.linear.z = feedback->actual.point.twist.linear.z - feedback->desired.point.twist.linear.z;
  error.twist.angular.x = feedback->actual.point.twist.angular.x - feedback->desired.point.twist.angular.x;
  error.twist.angular.y = feedback->actual.point.twist.angular.y - feedback->desired.point.twist.angular.y;
  error.twist.angular.z = feedback->actual.point.twist.angular.z - feedback->desired.point.twist.angular.z;

  // Compute the acceleration error
  error.acceleration.linear.x = feedback->actual.point.acceleration.linear.x - feedback->desired.point.acceleration.linear.x;
  error.acceleration.linear.y = feedback->actual.point.acceleration.linear.y - feedback->desired.point.acceleration.linear.y;
  error.acceleration.linear.z = feedback->actual.point.acceleration.linear.z - feedback->desired.point.acceleration.linear.z;
  error.acceleration.angular.x = feedback->actual.point.acceleration.angular.x - feedback->desired.point.acceleration.angular.x;
  error.acceleration.angular.y = feedback->actual.point.acceleration.angular.y - feedback->desired.point.acceleration.angular.y;
  error.acceleration.angular.z = feedback->actual.point.acceleration.angular.z - feedback->desired.point.acceleration.angular.z;

  // Compute the wrench error
  error.wrench.force.x = feedback->actual.point.wrench.force.x - feedback->desired.point.wrench.force.x;
  error.wrench.force.y = feedback->actual.point.wrench.force.y - feedback->desired.point.wrench.force.y;
  error.wrench.force.z = feedback->actual.point.wrench.force.z - feedback->desired.point.wrench.force.z;
  error.wrench.torque.x = feedback->actual.point.wrench.torque.x - feedback->desired.point.wrench.torque.x;
  error.wrench.torque.y = feedback->actual.point.wrench.torque.y - feedback->desired.point.wrench.torque.y;
  error.wrench.torque.z = feedback->actual.point.wrench.torque.z - feedback->desired.point.wrench.torque.z;

  // Compute the wrench derivative error
  error.wrench_derivative.force.x = feedback->actual.point.wrench_derivative.force.x - feedback->desired.point.wrench_derivative.force.x;
  error.wrench_derivative.force.y = feedback->actual.point.wrench_derivative.force.y - feedback->desired.point.wrench_derivative.force.y;
  error.wrench_derivative.force.z = feedback->actual.point.wrench_derivative.force.z - feedback->desired.point.wrench_derivative.force.z;
  error.wrench_derivative.torque.x = feedback->actual.point.wrench_derivative.torque.x - feedback->desired.point.wrench_derivative.torque.x;
  error.wrench_derivative.torque.y = feedback->actual.point.wrench_derivative.torque.y - feedback->desired.point.wrench_derivative.torque.y;
  error.wrench_derivative.torque.z = feedback->actual.point.wrench_derivative.torque.z - feedback->desired.point.wrench_derivative.torque.z;

  return error;
}

bool TaskSpaceReferenceGenerator::verify_frame_names_(const std::string& motion_frame, const std::string& wrench_frame) const
{
  if (robot_description_.find(motion_frame) == std::string::npos)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "The motion frame `%s` does not correspond to a robot's frame.", motion_frame.c_str());
    return false;
  }

  if (robot_description_.find(wrench_frame) == std::string::npos)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "The wrench frame `%s` does not correspond to a robot's frame.", wrench_frame.c_str());
    return false;
  }
  return true;
}

}  // namespace task_space_reference_generator

PLUGINLIB_EXPORT_CLASS(task_space_reference_generator::TaskSpaceReferenceGenerator, controller_interface::ControllerInterface)
