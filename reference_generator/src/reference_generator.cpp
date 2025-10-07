/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   reference_generator.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 4, 2025
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <rclcpp/logging.hpp>

#include "reference_generator/reference_generator.hpp"

namespace reference_generator
{

ReferenceGenerator::ReferenceGenerator(TrajectoryInfo& trajectory_info_base) : trajectory_info_(&trajectory_info_base, [](TrajectoryInfo*) {}) {}

controller_interface::CallbackReturn ReferenceGenerator::on_init()
{
  // Initialize the internal state of the controller
  state_ = State::ONLINE_REFERENCE;
  event_buffer_.writeFromNonRT(Event::NONE);

  try
  {
    // Create the parameter listener and get the parameters
    parameter_handler_ = std::make_shared<reference_generator::ParamListener>(get_node());
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "Exception thrown during init stage with message: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // Number of joints to control is fixed after initialization
  num_joints_ = parameter_handler_->get_params().joints.size();

  // The kinematics_interface requires the node to have the `robot_description` parameter. When working with a real robot, this parameter can be set
  // directly from the launch file. However, in Ignition Gazebo simulation, this is not possible. Therefore, when using simulation, the
  // `robot_description` parameter must be declared as a parameter of the reference generator node.
  if (!get_node()->has_parameter("robot_description"))
  {
    // Retrieve the robot description from the node robot_state_publisher
    std::shared_ptr<rclcpp::SyncParametersClient> parameters_client =
        std::make_shared<rclcpp::SyncParametersClient>(get_node(), "robot_state_publisher");
    std::chrono::duration<int, std::milli> ms(1000);
    while (!parameters_client->wait_for_service(ms))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_node()->get_logger(), "Service not available, waiting again...");
    }

    robot_description_ = parameters_client->get_parameter<std::string>("robot_description");

    // Declaring the robot description as parameter of this node
    get_node()->declare_parameter("robot_description", robot_description_);
  }
  else
  {
    get_node()->get_parameter("robot_description", robot_description_);
  }

  // Initializing internal variables from the node parameters
  task_space_reference_frame_ = parameter_handler_->get_params().task_space_reference_frame;
  wrench_reference_frame_ = parameter_handler_->get_params().wrench_reference_frame;
  tip_link_ = parameter_handler_->get_params().kinematics.tip;
  state_interfaces_names_ = parameter_handler_->get_params().state_interfaces;
  is_publish_desired_ = parameter_handler_->get_params().publish_task_space_reference.enable;

  if (!verify_link_name_(tip_link_) || !verify_link_name_(task_space_reference_frame_) || !verify_link_name_(wrench_reference_frame_))
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Load the kinematics plugin
  try
  {
    kinematics_loader_ = std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
        parameter_handler_->get_params().kinematics.plugin_package, "kinematics_interface::KinematicsInterface");
    kinematics_ = kinematics_loader_->createSharedInstance(parameter_handler_->get_params().kinematics.plugin_name);
    if (!kinematics_->initialize(get_node()->get_node_parameters_interface(), tip_link_))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "The Kinematics plugin '%s' failed to initialize. Using the link '%s' as tip link.",
                   parameter_handler_->get_params().kinematics.plugin_name.c_str(), tip_link_.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception while loading the Kinematics plugin '%s': '%s'",
                 parameter_handler_->get_params().kinematics.plugin_name.c_str(), ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  publish_frequency_ = parameter_handler_->get_params().publish_task_space_reference.frequency;

  acg_hardware_interface_facade::StateInterfaceNamesOverrideConfig state_interfaces_names_override;
  state_interfaces_names_override.position_state_interfaces = parameter_handler_->get_params().state_interfaces_names_override.position;
  state_interfaces_names_override.velocity_state_interfaces = parameter_handler_->get_params().state_interfaces_names_override.velocity;
  state_interfaces_names_override.acceleration_state_interfaces = parameter_handler_->get_params().state_interfaces_names_override.acceleration;
  state_interfaces_names_override.effort_state_interfaces = parameter_handler_->get_params().state_interfaces_names_override.effort;

  state_reader_.configure_state_interfaces(state_interfaces_names_, parameter_handler_->get_params().joints,
                                           parameter_handler_->get_params().robot_name, state_interfaces_names_override);

  // Check if the joint position state interface is available. Joint position is mandatory for the forward kinematics
  if (!state_reader_.has_joint_position_state_interface())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "The joint position state interface is not available.");
    return controller_interface::CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ReferenceGenerator::on_configure(const rclcpp_lifecycle::State&)
{
  // Initialize the internal variables of the controller
  robot_joint_state_ = acg_hardware_interface_facade::RobotJointState{};

  // Initializing the periodic real-time publisher. It publishes the reference so that it can be visualized in RViz
  periodic_reference_publisher_ = std::make_shared<acg_diagnostics::PeriodicPublisher<geometry_msgs::msg::PoseStamped>>(
      get_node(), "~/desired", rclcpp::SystemDefaultsQoS(), publish_frequency_);

  // Assign and initialize the internal variables that can vary according to the robot's configuration
  robot_joint_state_.positions.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
  if (state_reader_.has_joint_velocity_state_interface())
  {
    robot_joint_state_.velocities.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
  }
  if (state_reader_.has_joint_acceleration_state_interface())
  {
    robot_joint_state_.accelerations.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
  }
  if (state_reader_.has_joint_effort_state_interface())
  {
    robot_joint_state_.efforts.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ReferenceGenerator::on_activate(const rclcpp_lifecycle::State&)
{
  state_ = State::ONLINE_REFERENCE;
  event_buffer_.writeFromNonRT(Event::NONE);

  // Assigning the state interfaces given to this controller to the internal state reader
  state_reader_.assign_loaned_state_interfaces(ControllerInterfaceBase::state_interfaces_);

  // Write the current position of the joints from the robot
  state_reader_.read_state_interfaces(robot_joint_state_);

  // Log in the debug level the current joint state on the console
  RCLCPP_DEBUG(get_node()->get_logger(), "Joint position during activation:");
  for (std::size_t i = 0; i < robot_joint_state_.positions.size(); i++)
  {
    RCLCPP_DEBUG(get_node()->get_logger(), "Joint %ld: %f", i, robot_joint_state_.positions[i]);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ReferenceGenerator::state_interface_configuration() const
{
  // Configure the state interface for the joint space reference
  if (state_interfaces_names_.size() == 0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No state interface available.");
    return { controller_interface::interface_configuration_type::NONE, {} };
  }

  return { controller_interface::interface_configuration_type::INDIVIDUAL, state_reader_.available_state_interfaces() };
}

controller_interface::return_type ReferenceGenerator::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  // Get event from the real_time buffer
  Event event = *event_buffer_.readFromRT();

  // Update the current state based on the event
  switch (state_)
  {
    case State::ONLINE_REFERENCE:
      // Update the current state based on the event
      if (event == Event::TRAJECTORY_ACCEPTED)
      {
        RCLCPP_DEBUG(get_node()->get_logger(), "State changed from ONLINE_REFERENCE to TRAJECTORY_EXECUTION");

        // Read the trajectory info from the non-real-time context
        read_trajectory_info_from_non_rt();
        state_ = State::TRAJECTORY_EXECUTION;
      }

      if (event == Event::NEW_REFERENCE_PUBLISHED)
      {
        // Set the reference to the one received from the subscriber callback
        // The joint space reference is already checked in the subscriber callback, while
        // the task space reference is obtained through the forward kinematics.
        read_references_from_non_rt();
        state_ = State::ONLINE_REFERENCE;
      }
      break;
    case State::TRAJECTORY_EXECUTION:
      // Update the current state based on the event

      if (event == Event::TRAJECTORY_COMPLETED || event == Event::TRAJECTORY_CANCELLED)
      {
        // If the trajectory is completed or cancelled, the controller goes back to the ONLINE_REFERENCE state with the last reference
        // saved from the trajectory execution

        RCLCPP_DEBUG(get_node()->get_logger(), "State changed from TRAJECTORY_EXECUTION to ONLINE_REFERENCE");
        state_ = State::ONLINE_REFERENCE;
      }
      else if (event == Event::NEW_REFERENCE_PUBLISHED)
      {
        abort_trajectory("The joint trajectory execution has been aborted because a new reference has been received");

        // Set the reference as the one received from the topic.
        // The joint space reference is already checked in the subscriber callback, while
        // the task space reference is obtained through the forward kinematics.
        read_references_from_non_rt();

        RCLCPP_DEBUG(get_node()->get_logger(), "State changed from TRAJECTORY_EXECUTION to ONLINE_REFERENCE");
        state_ = State::ONLINE_REFERENCE;
      }
      else if (event == Event::TRAJECTORY_ACCEPTED)
      {
        // A new trajectory has been accepted. Retrieve the trajectory info from the non-real-time context
        read_trajectory_info_from_non_rt();
        state_ = State::TRAJECTORY_EXECUTION;
      }
      break;

    default:
      // If the state is not defined, return an error
      RCLCPP_ERROR(get_node()->get_logger(), "The FSM state is not defined");
      break;
  }

  // Reset the event
  // TODO: Need a lock-free write here. This is a known issue of real-time buffer.
  // See https://github.com/ros-controls/ros2_controllers/issues/168 and https://github.com/ros-controls/realtime_tools/issues/14
  event_buffer_.writeFromNonRT(Event::NONE);

  // Switch case to handle states
  switch (state_)
  {
    case State::ONLINE_REFERENCE:
      // Joint- and task- space commands are computed by the callbacks when an on-line reference is received:
      // in this state, the reference generator only writes to command interfaces
      break;

    case State::TRAJECTORY_EXECUTION:
      on_handle_trajectory_execution(time, period);
      break;

    default:
      // If the state is not defined, return an error
      RCLCPP_ERROR(get_node()->get_logger(), "The FSM state is not defined");
      break;
  }

  write_references_to_command_interfaces();

  // Publish the marker in RViz.
  if (is_publish_desired_)
  {
    publish_reference_pose(time);
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn ReferenceGenerator::on_deactivate(const rclcpp_lifecycle::State&)
{
  read_trajectory_info_from_non_rt();
  abort_trajectory("Trajectory execution has been aborted because the controller has been deactivated");
  ControllerInterfaceBase::release_interfaces();
  return CallbackReturn::SUCCESS;
}

void ReferenceGenerator::on_handle_trajectory_execution(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  // Creating and publishing the trajectory feedback
  calculate_and_publish_feedback(time);

  // Getting the index of the next trajectory waypoint, according to the current time of the trajectory and the timestamp of the waypoints
  const std::size_t next_trajectory_index{ get_next_trajectory_index() };

  // If the timestamp of the first waypoint is strictly greater than the controller's internal time, it means that the
  // trajectory execution should not begin immediately. Therefore, the controller must wait for the first waypoint
  // to be reached before executing the trajectory.
  if (next_trajectory_index > 0)
  {
    if (trajectory_info_->current_time_.seconds() > 0.0)
    {
      trajectory_info_->first_point_executed_ = true;
    }
    // Given the first waypoint whose timestamp is greater than the controller's internal time, command the previous waypoint
    trajectory_info_->trajectory_index_ = next_trajectory_index - 1;

    // Update the next joint space reference and task space reference to command from the trajectory
    if (!update_next_reference_from_trajectory())
    {
      abort_trajectory("The trajectory execution has been aborted because the next reference could not be updated");
      return;
    }

    // Verify if the trajectory execution has ended
    handle_trajectory_completed();
  }

  // Update the trajectory time
  trajectory_info_->current_time_ += period;
}

void ReferenceGenerator::trigger_trajectory_completed_event()
{
  // Write the event to the real-time buffer
  event_buffer_.writeFromNonRT(Event::TRAJECTORY_COMPLETED);
}

void ReferenceGenerator::trigger_trajectory_accepted_event()
{
  // Write the event to the real-time buffer
  event_buffer_.writeFromNonRT(Event::TRAJECTORY_ACCEPTED);
}

void ReferenceGenerator::trigger_trajectory_cancelled_event()
{
  // Write the event to the real-time buffer
  event_buffer_.writeFromNonRT(Event::TRAJECTORY_CANCELLED);
}

void ReferenceGenerator::trigger_publish_on_topic_event()
{
  // Write the event to the real-time buffer
  event_buffer_.writeFromNonRT(Event::NEW_REFERENCE_PUBLISHED);
}

bool ReferenceGenerator::verify_link_name_(const std::string& link_name) const
{
  if (link_name.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "The link name is empty.");
    return false;
  }

  if (robot_description_.find(link_name) == std::string::npos)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "The link name '%s' is not present in the robot description.", link_name.c_str());
    return false;
  }
  return true;
}

}  // namespace reference_generator
