/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   joint_space_reference_generator.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Nov 18, 2024
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <rclcpp/logging.hpp>
#include <rclcpp_action/create_server.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <acg_common_libraries/diagnostics.hpp>
#include <acg_common_libraries/interpolation.hpp>
#include <acg_common_libraries/message_utilities.hpp>

#include "reference_generator/joint_space_reference_generator.hpp"

namespace joint_space_reference_generator
{

JointSpaceReferenceGenerator::JointSpaceReferenceGenerator() : ReferenceGenerator(trajectory_info_) {}

controller_interface::CallbackReturn JointSpaceReferenceGenerator::on_init()
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
    parameter_handler_ = std::make_shared<joint_space_reference_generator::ParamListener>(get_node());
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "Exception thrown during init stage with message: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // Setting the command interface names override struct based on the configuration file
  acg_hardware_interface_facade::CommandInterfaceNamesOverrideConfig command_interface_names_override;
  command_interface_names_override.joint_position_interface_names = parameter_handler_->get_params().command_interfaces_names_override.joint_position;
  command_interface_names_override.joint_velocity_interface_names = parameter_handler_->get_params().command_interfaces_names_override.joint_velocity;
  command_interface_names_override.joint_acceleration_interface_names =
      parameter_handler_->get_params().command_interfaces_names_override.joint_acceleration;
  command_interface_names_override.joint_effort_interface_names = parameter_handler_->get_params().command_interfaces_names_override.joint_effort;
  command_interface_names_override.joint_wrench_interface_names = parameter_handler_->get_params().command_interfaces_names_override.joint_wrench;

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
  command_writer_.configure_interfaces(parameter_handler_->get_params().joints, parameter_handler_->get_params().joint_space_command_interfaces,
                                       parameter_handler_->get_params().task_space_command_interfaces,
                                       parameter_handler_->get_params().joint_space_command_controller,
                                       parameter_handler_->get_params().task_space_command_controller, command_interface_names_override);

  // Initialize robot kinematics
  robot_kinematics_.initialize(num_joints_, ReferenceGenerator::kinematics_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointSpaceReferenceGenerator::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  // Call the base class implementation
  controller_interface::CallbackReturn ret = ReferenceGenerator::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Initialize the internal joint reference with NaN variables
  if (command_writer_.has_joint_position_interface() || command_writer_.has_task_space_pose_interface() ||
      command_writer_.has_task_space_twist_interface())
  {
    joint_reference_.positions.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
  }
  if (command_writer_.has_joint_velocity_interface() || command_writer_.has_task_space_twist_interface())
  {
    joint_reference_.velocities.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
  }
  if (command_writer_.has_joint_acceleration_interface())
  {
    joint_reference_.accelerations.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
  }
  if (command_writer_.has_joint_effort_interface())
  {
    joint_reference_.effort.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
  }
  acg_message_utilities::clear(joint_reference_.wrench);

  // Initialize feedback with NaN values
  feedback_ = std::make_shared<JointTrajFeedback>();
  if (command_writer_.has_joint_position_interface() || command_writer_.has_task_space_pose_interface() ||
      command_writer_.has_task_space_twist_interface())
  {
    feedback_->desired.point.positions.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
    feedback_->error.point.positions.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
  }
  if (command_writer_.has_joint_velocity_interface() || command_writer_.has_task_space_twist_interface())
  {
    feedback_->desired.point.velocities.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
    feedback_->error.point.velocities.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
  }
  if (command_writer_.has_joint_acceleration_interface())
  {
    feedback_->desired.point.accelerations.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
    feedback_->error.point.accelerations.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
  }
  if (command_writer_.has_joint_effort_interface())
  {
    feedback_->desired.point.effort.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
    feedback_->error.point.effort.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
  }

  // Joint positions are guaranteed to be available
  feedback_->actual.point.positions.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
  if (state_reader_.has_joint_velocity_state_interface())
  {
    feedback_->actual.point.velocities.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
  }
  if (state_reader_.has_joint_acceleration_state_interface())
  {
    feedback_->actual.point.accelerations.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
  }
  if (state_reader_.has_joint_effort_state_interface())
  {
    feedback_->actual.point.effort.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
  }

  // Reserve space for the points in the trajectory, so that the vector doesn't have to reallocate memory during execution
  current_traj_point_.point.positions.reserve(num_joints_);
  current_traj_point_.point.velocities.reserve(num_joints_);
  current_traj_point_.point.accelerations.reserve(num_joints_);
  current_traj_point_.point.effort.reserve(num_joints_);
  current_traj_point_.point.wrench_frame.reserve(ReferenceGenerator::STRING_INITIAL_CAPACITY_);

  next_traj_point_.point.positions.reserve(num_joints_);
  next_traj_point_.point.velocities.reserve(num_joints_);
  next_traj_point_.point.accelerations.reserve(num_joints_);
  next_traj_point_.point.effort.reserve(num_joints_);
  next_traj_point_.point.wrench_frame.reserve(ReferenceGenerator::STRING_INITIAL_CAPACITY_);

  joint_space_trajectory_action_server_ = rclcpp_action::create_server<acg_control_msgs::action::FollowJointTrajectory>(
      get_node(), "~/follow_joint_trajectory",
      std::bind(&JointSpaceReferenceGenerator::handle_goal_, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointSpaceReferenceGenerator::handle_cancel_, this, std::placeholders::_1),
      std::bind(&JointSpaceReferenceGenerator::handle_accepted_, this, std::placeholders::_1));

  // Set up QoS settings
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  joint_space_reference_subscriber_ = get_node()->create_subscription<acg_control_msgs::msg::JointWrenchPoint>(
      "~/reference", qos, std::bind(&JointSpaceReferenceGenerator::joint_space_reference_callback_, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointSpaceReferenceGenerator::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  // Call the base class implementation
  controller_interface::CallbackReturn ret = ReferenceGenerator::on_activate(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Assign the loaned command interfaces to the internal command writer
  command_writer_.assign_loaned_command_interfaces(ControllerInterfaceBase::command_interfaces_);

  // Initialize the joint space reference with the current joint state and joint space real-time buffer as well
  if (command_writer_.has_joint_position_interface() || command_writer_.has_task_space_pose_interface() ||
      command_writer_.has_task_space_twist_interface())
  {
    joint_reference_.positions = robot_joint_state_.positions;
  }
  // Setting velocities and accelerations to zero, because it is desired that the robot does not move when the controller is activated
  if (command_writer_.has_joint_velocity_interface() || command_writer_.has_task_space_twist_interface())
  {
    std::fill(joint_reference_.velocities.begin(), joint_reference_.velocities.end(), 0.0);
  }
  if (command_writer_.has_joint_acceleration_interface())
  {
    std::fill(joint_reference_.accelerations.begin(), joint_reference_.accelerations.end(), 0.0);
  }

  // Setting the effort to NaN, even if the effort interface is used because there is no general interpretation of the effort values
  // read from the robot state interface
  if (command_writer_.has_joint_effort_interface())
  {
    std::fill(joint_reference_.effort.begin(), joint_reference_.effort.end(), std::numeric_limits<double>::quiet_NaN());
  }

  // Setting the wrench to 0, if available
  if (command_writer_.has_joint_wrench_interface())
  {
    joint_reference_.wrench = geometry_msgs::msg::Wrench();
  }
  else
  {
    acg_message_utilities::clear(joint_reference_.wrench);
  }

  // Initializing the task space reference

  // Setting all values to NaN
  acg_message_utilities::clear(task_space_reference_);
  task_space_reference_.motion_frame = ReferenceGenerator::task_space_reference_frame_;
  task_space_reference_.wrench_frame = ReferenceGenerator::wrench_reference_frame_;

  // Compute the pose of the task space reference even if the task space reference is not used so that it can be published
  robot_kinematics_.compute_forward_kinematics(robot_joint_state_.positions, ReferenceGenerator::tip_link_,
                                               ReferenceGenerator::task_space_reference_frame_, task_space_reference_.pose);

  // Initializing the task space reference with zero values
  if (command_writer_.has_task_space_twist_interface())
  {
    task_space_reference_.twist = geometry_msgs::msg::Twist();
  }
  if (command_writer_.has_task_space_acceleration_interface())
  {
    task_space_reference_.acceleration = geometry_msgs::msg::Accel();
  }
  if (command_writer_.has_task_space_wrench_interface())
  {
    task_space_reference_.wrench = geometry_msgs::msg::Wrench();
  }
  if (command_writer_.has_task_space_wrench_derivative_interface())
  {
    task_space_reference_.wrench_derivative = geometry_msgs::msg::Wrench();
  }

  joint_space_reference_buffer_.writeFromNonRT(std::make_shared<acg_control_msgs::msg::JointWrenchPoint>(joint_reference_));
  task_space_reference_buffer_.writeFromNonRT(std::make_shared<acg_control_msgs::msg::TaskSpacePoint>(task_space_reference_));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration JointSpaceReferenceGenerator::command_interface_configuration() const
{
  // Configure the command interface for the joint space reference
  std::string joint_space_command_controller_name = parameter_handler_->get_params().joint_space_command_controller;
  std::string task_space_command_controller_name = parameter_handler_->get_params().task_space_command_controller;

  if (joint_space_command_controller_name.empty() && task_space_command_controller_name.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No command controller specified in the configuration file.");
    return { controller_interface::interface_configuration_type::NONE, {} };
  }

  return { controller_interface::interface_configuration_type::INDIVIDUAL, command_writer_.available_interfaces() };
}

void JointSpaceReferenceGenerator::abort_trajectory(const std::string& error_string)
{
  // If the goal handle is still valid, abort the goal
  if (trajectory_info_.goal_handle_)
  {
    std::shared_ptr<acg_control_msgs::action::FollowJointTrajectory_Result> result =
        std::make_shared<acg_control_msgs::action::FollowJointTrajectory::Result>();
    result->error_code = acg_control_msgs::action::FollowJointTrajectory::Result::TRAJECTORY_ABORTED;
    result->error_string = error_string;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<acg_control_msgs::action::FollowJointTrajectory>> goal_handle = trajectory_info_.goal_handle_;
    goal_handle->abort(result);
    goal_handle.reset();
    trajectory_info_ = JointTrajectoryInfo();
  }
  trigger_trajectory_cancelled_event();
}

void JointSpaceReferenceGenerator::read_references_from_non_rt()
{
  // Set the reference to the one received from the subscriber callback
  // The joint space reference is already checked in the subscriber callback
  joint_reference_ = **joint_space_reference_buffer_.readFromRT();
  task_space_reference_ = **task_space_reference_buffer_.readFromRT();
}

void JointSpaceReferenceGenerator::write_references_to_command_interfaces()
{
  // Write the joint space reference to the command interfaces
  command_writer_.write_to_command_interfaces(joint_reference_);

  // Write the task space reference to the command interfaces
  command_writer_.write_to_command_interfaces(task_space_reference_);
}

void JointSpaceReferenceGenerator::calculate_and_publish_feedback(const rclcpp::Time& time)
{
  feedback_->header.stamp = time;
  state_reader_.read_state_interfaces(robot_joint_state_);

  if (!trajectory_info_.first_point_executed_)
  {
    // Since the trajectory has not started yet (because the user delayed it) the reference is set to NaN
    acg_message_utilities::clear(feedback_->desired.point);
  }
  else
  {
    feedback_->desired.point = joint_reference_;
  }

  // Update actual trajectory feedback. It is guaranteed that robot_joint_state_ and feedback_->actual.point fields are the same size
  feedback_->actual.point.positions = robot_joint_state_.positions;
  feedback_->actual.point.velocities = robot_joint_state_.velocities;
  feedback_->actual.point.accelerations = robot_joint_state_.accelerations;
  feedback_->actual.point.effort = robot_joint_state_.efforts;
  acg_message_utilities::clear(feedback_->actual.point.wrench);

  // Update error trajectory feedback
  compute_feedback_error_(feedback_->desired.point, feedback_->actual.point, feedback_->error.point);

  // Publish feedback
  trajectory_info_.goal_handle_->publish_feedback(feedback_);
}

std::size_t JointSpaceReferenceGenerator::get_next_trajectory_index()
{
  // Getting a reference to the joint trajectory (so it is not copied)
  const acg_control_msgs::msg::JointTrajectory& joint_trajectory = trajectory_info_.goal_handle_->get_goal()->trajectory;

  // Getting the index of the last trajectory waypoint
  std::size_t next_trajectory_index{ trajectory_info_.trajectory_index_ };

  // Getting the current time of the trajectory
  const rclcpp::Time joint_trajectory_time{ trajectory_info_.current_time_ };

  // The next waypoint to command is the last one whose timestamp is less than the controller's internal time
  rclcpp::Duration time_from_start{ joint_trajectory.points[next_trajectory_index].time_from_start };

  while (next_trajectory_index < joint_trajectory.points.size() && time_from_start.seconds() <= joint_trajectory_time.seconds())
  {
    next_trajectory_index++;
    time_from_start = rclcpp::Duration(joint_trajectory.points[next_trajectory_index].time_from_start);
  }

  return next_trajectory_index;
}

bool JointSpaceReferenceGenerator::update_next_reference_from_trajectory()
{
  const acg_control_msgs::msg::JointTrajectory& joint_trajectory = trajectory_info_.goal_handle_->get_goal()->trajectory;

  current_traj_point_ = joint_trajectory.points[trajectory_info_.trajectory_index_];

  // Updating the wrench frame of the current point, if it is different from the desired wrench frame
  state_reader_.read_state_interfaces(robot_joint_state_);
  try
  {
    ensure_wrench_frame_or_clear_(ReferenceGenerator::wrench_reference_frame_, robot_joint_state_.positions, current_traj_point_.point);
  }
  catch (const std::runtime_error& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error while ensuring the wrench frame: %s", e.what());
    return false;
  }

  // If it is the last point of the trajectory, the reference is the last point and no interpolation is needed
  if (trajectory_info_.trajectory_index_ == joint_trajectory.points.size() - 1)
  {
    joint_reference_ = current_traj_point_.point;
  }
  else
  {
    next_traj_point_ = joint_trajectory.points[trajectory_info_.trajectory_index_ + 1];

    // Update the wrench frame of the next point, if it is different from the desired wrench frame
    try
    {
      ensure_wrench_frame_or_clear_(ReferenceGenerator::wrench_reference_frame_, robot_joint_state_.positions, next_traj_point_.point);
    }
    catch (const std::runtime_error& e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Error while ensuring the wrench frame: %s", e.what());
      return false;
    }

    try
    {
      acg_interpolation::linearly_interpolate(current_traj_point_, next_traj_point_, trajectory_info_.current_time_.seconds(), joint_reference_);
    }
    catch (const std::runtime_error& e)
    {
      return false;
    }
  }

  // Compute the forward kinematics and write the task space reference in the task space real-time buffer
  try
  {
    compute_task_space_point_from_joint_space_(joint_reference_.positions, joint_reference_.velocities);
  }
  catch (const std::runtime_error& e)
  {
    return false;
  }

  return true;
}

void JointSpaceReferenceGenerator::handle_trajectory_completed()
{
  const acg_control_msgs::msg::JointTrajectory& joint_trajectory = trajectory_info_.goal_handle_->get_goal()->trajectory;
  if (trajectory_info_.trajectory_index_ == joint_trajectory.points.size() - 1)
  {
    std::shared_ptr<acg_control_msgs::action::FollowJointTrajectory_Result> result =
        std::make_shared<acg_control_msgs::action::FollowJointTrajectory::Result>();
    result->error_code = acg_control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
    trajectory_info_.goal_handle_->succeed(result);

    trajectory_info_.goal_handle_.reset();

    // TODO: Need a lock-free write here. This is a known issue of real-time buffer.
    // See https://github.com/ros-controls/ros2_controllers/issues/168 and https://github.com/ros-controls/realtime_tools/issues/14
    trigger_trajectory_completed_event();
    trajectory_info_ = JointTrajectoryInfo();
    trajectory_info_buffer_.writeFromNonRT(trajectory_info_);
  }
}

void JointSpaceReferenceGenerator::publish_reference_pose(const rclcpp::Time& time)
{
  // Publish the reference so that it can be visualized in RViz.
  periodic_reference_publisher_->publish(
      acg_message_utilities::build_pose_stamped_msg(task_space_reference_.pose, ReferenceGenerator::task_space_reference_frame_, time), time);
}

void JointSpaceReferenceGenerator::read_trajectory_info_from_non_rt()
{
  // Get the trajectory info from the real-time buffer
  trajectory_info_ = *trajectory_info_buffer_.readFromRT();
}

rclcpp_action::GoalResponse JointSpaceReferenceGenerator::handle_goal_(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const acg_control_msgs::action::FollowJointTrajectory::Goal> goal)
{
  if (goal->trajectory.points.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "The received joint space trajectory has been rejected because the number of points in the trajectory is %ld",
                 goal->trajectory.points.size());
    return rclcpp_action::GoalResponse::REJECT;
  }

  const rclcpp::Time trajectory_start_time = rclcpp::Time(goal->trajectory.header.stamp);

  // If the starting time is set to 0.0, it means the controller should begin executing the trajectory immediately; this is a convention.
  // Otherwise we check if the task space trajectory ends before the current time, in which case it can be ignored. In the case described above
  // (starting time = 0.0), this check is not done because trajectory end-time is always less than the current time.
  if (trajectory_start_time.seconds() != 0.0)
  {
    const rclcpp::Time trajectory_end_time = trajectory_start_time + rclcpp::Duration(goal->trajectory.points.back().time_from_start);
    const rclcpp::Time curr_time{ get_node()->now() };
    if (trajectory_end_time < curr_time)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Received trajectory with non-zero start time (%f) that ends in the past (%f). Current time is: %f",
                   trajectory_start_time.seconds(), trajectory_end_time.seconds(), curr_time.seconds());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  if (goal->trajectory.joint_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Empty joint names on incoming trajectory.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::vector<std::string, std::allocator<std::string>> joints = parameter_handler_->get_params().joints;
  if (goal->trajectory.joint_names.size() != joints.size())
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Incoming trajectory has %ld joint names, %ld was expected. The trajectory is expected to contain a reference for each "
                 "controller's joint.",
                 goal->trajectory.joint_names.size(), joints.size());
    return rclcpp_action::GoalResponse::REJECT;
  }

  for (std::size_t i = 0; i < goal->trajectory.joint_names.size(); ++i)
  {
    if (goal->trajectory.joint_names[i] != joints[i])
    {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Incoming joint %s doesn't match the controller's joint %s. Incoming trajectory is expected to have the joint names "
                   "provided in the same order as the controller's joints.",
                   goal->trajectory.joint_names[i].c_str(), joints[i].c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  bool no_wrench_frame{ false };

  for (std::size_t i = 0; i < goal->trajectory.points.size(); i++)
  {
    if ((i > 0) && (rclcpp::Duration(goal->trajectory.points[i].time_from_start) <= rclcpp::Duration(goal->trajectory.points[i - 1].time_from_start)))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Time between points %zu and %zu is not strictly increasing, it is %f and %f respectively", i - 1, i,
                   rclcpp::Duration(goal->trajectory.points[i - 1].time_from_start).seconds(),
                   rclcpp::Duration(goal->trajectory.points[i].time_from_start).seconds());
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!check_joint_space_trajectory_point_(goal->trajectory.points[i].point, num_joints_))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "The joint space trajectory point with index %zu has been rejected", i);
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Check if the wrench frame is empty. Note that the wrench frame is not mandatory.
    if (goal->trajectory.points[i].point.wrench_frame.empty())
    {
      no_wrench_frame = true;
    }
  }

  if (no_wrench_frame)
  {
    RCLCPP_WARN(get_node()->get_logger(),
                "No wrench frame for at least one point of the incoming task space trajectory. %s frame is used as default for those points.",
                ReferenceGenerator::wrench_reference_frame_.c_str());
  }

  RCLCPP_INFO(get_node()->get_logger(), "Received a new joint space trajectory");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
};

rclcpp_action::CancelResponse JointSpaceReferenceGenerator::handle_cancel_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<acg_control_msgs::action::FollowJointTrajectory>> /* goal_handle */)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received request to cancel joint space trajectory goal");

  trigger_trajectory_cancelled_event();
  return rclcpp_action::CancelResponse::ACCEPT;
};

void JointSpaceReferenceGenerator::handle_accepted_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<acg_control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received request to execute joint space trajectory goal");

  // Create a new trajectory info object and store the goal handle
  JointTrajectoryInfo trajectory_info;
  trajectory_info.goal_handle_ = goal_handle;

  // TODO: Need a lock-free write here. This is a known issue of real-time buffer.
  // See https://github.com/ros-controls/ros2_controllers/issues/168 and https://github.com/ros-controls/realtime_tools/issues/14
  trajectory_info_buffer_.writeFromNonRT(trajectory_info);

  trigger_trajectory_accepted_event();
};

void JointSpaceReferenceGenerator::joint_space_reference_callback_(const std::shared_ptr<acg_control_msgs::msg::JointWrenchPoint> msg)
{
  // Check if the joint space reference is valid
  std::vector<std::string> joint_space_command_interfaces = parameter_handler_->get_params().joint_space_command_interfaces;
  if (check_joint_space_trajectory_point_(*msg, num_joints_))
  {
    RCLCPP_INFO(get_node()->get_logger(), "The joint space reference has been accepted");
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "The joint space reference has been rejected");
    return;
  }

  // Convert the wrench reference frame to the one specified in the configuration file
  if (msg->wrench_frame.empty())
  {
    RCLCPP_WARN(get_node()->get_logger(),
                "No wrench reference frame specified in the joint space reference message. Assuming the wrench is expressed in the desired wrench "
                "frame: %s",
                ReferenceGenerator::wrench_reference_frame_.c_str());
    msg->wrench_frame = ReferenceGenerator::wrench_reference_frame_;
  }
  else if (msg->wrench_frame != ReferenceGenerator::wrench_reference_frame_)
  {
    acg_kinematics::transform_wrench_frame(*kinematics_, msg->positions, ReferenceGenerator::wrench_reference_frame_, msg->wrench_frame, msg->wrench);
    msg->wrench_frame = ReferenceGenerator::wrench_reference_frame_;
  }

  // Write the joint space reference to the real-time buffer
  joint_space_reference_buffer_.writeFromNonRT(msg);

  compute_task_space_point_from_joint_space_(msg->positions, msg->velocities);

  // Write the task space reference to the task space real-time buffer
  task_space_reference_buffer_.writeFromNonRT(std::make_shared<acg_control_msgs::msg::TaskSpacePoint>(task_space_reference_));

  trigger_publish_on_topic_event();
};

void JointSpaceReferenceGenerator::compute_feedback_error_(const acg_control_msgs::msg::JointWrenchPoint& desired,
                                                           const acg_control_msgs::msg::JointWrenchPoint& actual,
                                                           acg_control_msgs::msg::JointWrenchPoint& error)
{
  std::function<void(const std::vector<double>&, const std::vector<double>&, std::vector<double>&)> compute_error =
      [](const std::vector<double>& des, const std::vector<double>& act, std::vector<double>& err)
  {
    if (des.size() == act.size())
    {
      std::transform(des.begin(), des.end(), act.begin(), err.begin(), std::minus<double>());
    }
  };

  // Compute joint error
  compute_error(desired.positions, actual.positions, error.positions);
  compute_error(desired.velocities, actual.velocities, error.velocities);
  compute_error(desired.accelerations, actual.accelerations, error.accelerations);
  compute_error(actual.effort, desired.effort, error.effort);

  // If the desired wrench or the actual wrench is NaN, the error will be NaN
  error.wrench.force.x = desired.wrench.force.x - actual.wrench.force.x;
  error.wrench.force.y = desired.wrench.force.y - actual.wrench.force.y;
  error.wrench.force.z = desired.wrench.force.z - actual.wrench.force.z;
  error.wrench.torque.x = desired.wrench.torque.x - actual.wrench.torque.x;
  error.wrench.torque.y = desired.wrench.torque.y - actual.wrench.torque.y;
  error.wrench.torque.z = desired.wrench.torque.z - actual.wrench.torque.z;
}

bool JointSpaceReferenceGenerator::check_joint_space_trajectory_point_(const acg_control_msgs::msg::JointWrenchPoint& joint_space_point,
                                                                       const std::size_t num_joints) const
{
  // Create a lambda function to check the interface:
  // If the interface is not available, the corresponding vector in the joint space point must be empty
  // If the interface is available, the corresponding vector in the joint space point must have the same size as the number of joints
  // Note that it is needed to capture the class instance, to log the error message
  std::function<bool(const bool, const std::vector<double>&, const std::string&, const std::size_t)> check_interface =
      [this](bool has_interface, const std::vector<double>& values, const std::string& interface_name, std::size_t num_joints)
  {
    if (has_interface)
    {
      if (values.size() != num_joints)
      {
        RCLCPP_ERROR(get_node()->get_logger(), "The number of joint %s in the joint space point is not equal to the number of joints",
                     interface_name.c_str());
        return false;
      }
    }
    else
    {
      if (!values.empty())
      {
        RCLCPP_ERROR(get_node()->get_logger(), "The joint %s in the joint space point are not empty", interface_name.c_str());
        return false;
      }
    }
    return true;
  };

  const bool has_position_interface{ command_writer_.has_joint_position_interface() || command_writer_.has_task_space_pose_interface() ||
                                     command_writer_.has_task_space_twist_interface() };
  const bool has_velocity_interface{ command_writer_.has_joint_velocity_interface() || command_writer_.has_task_space_twist_interface() };

  if (!check_interface(has_position_interface, joint_space_point.positions, "positions", num_joints) ||
      !check_interface(has_velocity_interface, joint_space_point.velocities, "velocities", num_joints) ||
      !check_interface(command_writer_.has_joint_acceleration_interface(), joint_space_point.accelerations, "accelerations", num_joints) ||
      !check_interface(command_writer_.has_joint_effort_interface(), joint_space_point.effort, "effort", num_joints))
  {
    return false;
  }

  if (command_writer_.has_joint_wrench_interface())
  {
    // Note that the empty wrench is not captured in the following check
    if (robot_description_.find(joint_space_point.wrench_frame) == std::string::npos)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "The wrench frame `%s` of the incoming joint space point doesn't correspond to a robot's frame.",
                   joint_space_point.wrench_frame.c_str());
      return false;
    }
  }

  return true;
}

void JointSpaceReferenceGenerator::compute_task_space_point_from_joint_space_(const std::vector<double>& positions,
                                                                              const std::vector<double>& velocities)
{
  // Clear the task space reference, so that the controller does not use the previous reference
  acg_message_utilities::clear(task_space_reference_);
  task_space_reference_.motion_frame = ReferenceGenerator::task_space_reference_frame_;
  task_space_reference_.wrench_frame = ReferenceGenerator::wrench_reference_frame_;

  // If positions are empty, the controller is not able to compute any forward kinematics, so the reference is NaN
  if (positions.empty())
  {
    return;
  }

  // Compute the forward kinematics and write the task space reference in the task space real-time buffer
  if (command_writer_.has_task_space_twist_interface() && !velocities.empty())
  {
    robot_kinematics_.compute_forward_kinematics(positions, velocities, ReferenceGenerator::tip_link_,
                                                 ReferenceGenerator::task_space_reference_frame_, task_space_reference_);
  }
  else
  {
    robot_kinematics_.compute_forward_kinematics(positions, ReferenceGenerator::tip_link_, ReferenceGenerator::task_space_reference_frame_,
                                                 task_space_reference_.pose);
  }

  if (command_writer_.has_task_space_wrench_interface() && command_writer_.has_joint_wrench_interface())
  {
    task_space_reference_.wrench = joint_reference_.wrench;
  }
}

void JointSpaceReferenceGenerator::ensure_wrench_frame_or_clear_(const std::string& desired_wrench_frame, const std::vector<double>& positions,
                                                                 acg_control_msgs::msg::JointWrenchPoint& point)
{
  if (command_writer_.has_joint_wrench_interface())
  {
    if (!point.wrench_frame.empty() && point.wrench_frame != desired_wrench_frame)
    {
      acg_kinematics::transform_wrench_frame(*ReferenceGenerator::kinematics_, positions, desired_wrench_frame, point.wrench_frame, point.wrench);
    }
  }
  else
  {
    acg_message_utilities::clear(point.wrench);
  }
}

}  // namespace joint_space_reference_generator

PLUGINLIB_EXPORT_CLASS(joint_space_reference_generator::JointSpaceReferenceGenerator, controller_interface::ControllerInterface)
