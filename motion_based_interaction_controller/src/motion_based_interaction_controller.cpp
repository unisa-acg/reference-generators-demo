/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   motion_based_interaction_controller.cpp
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Feb 4, 2025
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <eigen3/Eigen/Core>

#include <acg_common_libraries/message_utilities.hpp>

#include "motion_based_interaction_controller/motion_based_interaction_controller.hpp"

namespace motion_based_interaction_controller
{
controller_interface::CallbackReturn MotionBasedInteractionController::on_init()
{
  // Create the parameter listener and get the parameters
  try
  {
    parameter_handler_ = std::make_shared<motion_based_interaction_controller::ParamListener>(get_node());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error creating the parameter handler: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  parameters_ = parameter_handler_->get_params();

  // Get the number of joints from the parameter handler
  number_of_joints_ = parameters_.joints.size();

  // The kinematics interface requires the node to have the `robot_description` parameter.
  // While in simulation the `robot_description` is already a parameter of the control node,
  // when working with a real robot, this parameter can be set directly from the launch file.
  std::string robot_description;
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

    robot_description = parameters_client->get_parameter<std::string>("robot_description");

    // Declaring the robot description as parameter of this node
    get_node()->declare_parameter("robot_description", robot_description);
  }
  else
  {
    get_node()->get_parameter("robot_description", robot_description);
  }

  // Set the end-effector frame
  end_effector_frame_ = parameters_.kinematics.tip;

  // Load the kinematics plugin
  try
  {
    kinematics_loader_ = std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
        parameters_.kinematics.plugin_package, "kinematics_interface::KinematicsInterface");
    kinematics_ = kinematics_loader_->createSharedInstance(parameters_.kinematics.plugin_name);

    if (!kinematics_->initialize(get_node()->get_node_parameters_interface(), end_effector_frame_))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "The Kinematics plugin '%s' failed to initialize. Using the link '%s' as tip link.",
                   parameters_.kinematics.plugin_name.c_str(), end_effector_frame_.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception while loading the Kinematics plugin '%s': '%s'", parameters_.kinematics.plugin_name.c_str(),
                 ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Load the interaction filter plugin
  try
  {
    interaction_filter_loader_ = std::make_shared<pluginlib::ClassLoader<interaction_filters::InteractionFilterBase>>(
        "interaction_filter_base", "interaction_filters::InteractionFilterBase");

    interaction_filter_ = interaction_filter_loader_->createUniqueInstance(parameters_.interaction_filter.type);

    if (!interaction_filter_->initialize(get_node()->get_node_parameters_interface(), get_node()->get_node_logging_interface(),
                                         parameters_.interaction_filter.name))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "The Interaction filter plugin '%s' failed to initialize.", parameters_.interaction_filter.type.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception while loading the Interaction filter plugin '%s': '%s'",
                 parameters_.interaction_filter.type.c_str(), ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize state reader
  state_reader_ = std::make_shared<acg_hardware_interface_facade::StateReader>();

  // Initialize command writer
  command_writer_ = std::make_shared<acg_hardware_interface_facade::CommandWriter>();

  // Initialize the force/torque sensor reader
  force_torque_sensor_reader_ = std::make_shared<acg_hardware_interface_facade::ForceTorqueSensorReader>();

  // Initialize the reference reader
  reference_reader_ = std::make_shared<acg_hardware_interface_facade::ReferenceReader>();

  // Assign and initialize the internal variables that can vary according to the robot's configuration
  joint_space_state_.positions.assign(number_of_joints_, std::numeric_limits<double>::quiet_NaN());
  joint_space_state_.velocities.assign(number_of_joints_, std::numeric_limits<double>::quiet_NaN());

  // Initialize the force filter chain
  force_filter_chain_ = std::make_shared<filters::MultiChannelFilterChain<double>>("double");

  // Initialize the robot kinematics utility
  robot_kinematics_ = std::make_shared<acg_kinematics::RTKinematicsSolver>();
  robot_kinematics_->initialize(number_of_joints_, kinematics_);

  // Set the motion reference frame
  motion_reference_frame_ = parameters_.task_space_reference_frame;

  // Set the wrench reference frame
  wrench_reference_frame_ = parameters_.wrench_reference_frame;

  // Set the force/torque sensor frame
  force_torque_measure_frame_ = parameters_.force_torque_sensor.measure_frame;

  // Set the initial state of the controller
  controller_state_ = State::FILTERS_INITIALIZATION;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MotionBasedInteractionController::command_interface_configuration() const
{
  return { controller_interface::interface_configuration_type::INDIVIDUAL, command_writer_->available_interfaces() };
}

controller_interface::InterfaceConfiguration MotionBasedInteractionController::state_interface_configuration() const
{
  std::vector<std::string> state_interfaces_config_names{ state_reader_->available_state_interfaces() };

  std::vector<std::string> force_torque_sensor_iterfaces_names = force_torque_sensor_reader_->available_state_interfaces();
  state_interfaces_config_names.insert(state_interfaces_config_names.end(), force_torque_sensor_iterfaces_names.begin(),
                                       force_torque_sensor_iterfaces_names.end());
  return { controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names };
}

controller_interface::CallbackReturn MotionBasedInteractionController::on_configure(const rclcpp_lifecycle::State& /* previous_state */)
{
  // Configure state interfaces
  std::vector<std::string> state_interfaces;
  state_interfaces.emplace_back("position");
  state_interfaces.emplace_back("velocity");

  acg_hardware_interface_facade::StateInterfaceNamesOverrideConfig state_interfaces_names_override;
  state_interfaces_names_override.position_state_interfaces = parameters_.state_interfaces_names_override.position;
  state_interfaces_names_override.velocity_state_interfaces = parameters_.state_interfaces_names_override.velocity;
  if (!state_reader_->configure_state_interfaces(state_interfaces, parameters_.joints, parameters_.robot_name, state_interfaces_names_override))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error configuring state interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Configure force/torque sensor state interfaces
  std::vector<std::string> ft_sensor_state_interfaces_names_override = parameters_.ft_sensor_state_interfaces_names_override;
  if (!force_torque_sensor_reader_->configure_state_interfaces(parameters_.force_torque_sensor.name, ft_sensor_state_interfaces_names_override))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error configuring force/torque sensor state interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Configure command interfaces
  std::vector<std::string> command_interfaces = parameters_.command_interfaces;

  acg_hardware_interface_facade::CommandInterfaceNamesOverrideConfig command_interfaces_names_override;
  command_interfaces_names_override.task_space_pose_interface_names = parameters_.command_interfaces_names_override.pose;
  command_interfaces_names_override.task_space_twist_interface_names = parameters_.command_interfaces_names_override.twist;

  if (!command_writer_->configure_interfaces(std::vector<std::string>(), std::vector<std::string>(), command_interfaces, std::string(""),
                                             parameters_.task_space_command_controller, command_interfaces_names_override))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error configuring command interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Configure reference interfaces
  std::vector<std::string> reference_interfaces = parameters_.reference_interfaces;

  acg_hardware_interface_facade::CommandInterfaceNamesOverrideConfig reference_interfaces_names_override;
  reference_interfaces_names_override.task_space_pose_interface_names = parameters_.reference_interfaces_names_override.pose;
  reference_interfaces_names_override.task_space_twist_interface_names = parameters_.reference_interfaces_names_override.twist;
  reference_interfaces_names_override.task_space_wrench_interface_names = parameters_.reference_interfaces_names_override.wrench;

  if (!reference_reader_->configure_interfaces(std::vector<std::string>(), std::vector<std::string>(), reference_interfaces, std::string(""),
                                               get_node()->get_name(), reference_interfaces_names_override))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error configuring reference interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Configure the force filter chain
  if (parameters_.fixed_robot_orientation)
  {
    filter_chain_number_of_channels_ = FT_SENSOR_AXES + 1;
    if (!force_filter_chain_->configure(filter_chain_number_of_channels_, "filter_chain.fixed_orientation", get_node()->get_node_logging_interface(),
                                        get_node()->get_node_parameters_interface()))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "The filter chain hasn't been correctly configured.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  else
  {
    filter_chain_number_of_channels_ = GRAVITY_COMPENSATION_FILTER_CHANNELS;
    if (!force_filter_chain_->configure(filter_chain_number_of_channels_, "filter_chain.variable_orientation",
                                        get_node()->get_node_logging_interface(), get_node()->get_node_parameters_interface()))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "The filter chain hasn't been correctly configured.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionBasedInteractionController::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
{
  // Assigning the state interfaces given to this controller to the internal state reader
  if (!state_reader_->assign_loaned_state_interfaces(ControllerInterfaceBase::state_interfaces_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error assigning state interfaces to the state reader.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Assigning the force/torque sensor state interfaces given to this controller to the internal force/torque sensor reader
  if (!force_torque_sensor_reader_->assign_loaned_state_interfaces(ControllerInterfaceBase::state_interfaces_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error assigning force/torque sensor state interfaces to the force/torque sensor reader.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Assigning the command interfaces given to this controller to the internal command writer
  command_writer_->assign_loaned_command_interfaces(ControllerInterfaceBase::command_interfaces_);

  // Assigning the command interfaces given to this controller to the internal reference reader
  reference_reader_->assign_command_interfaces(on_export_reference_interfaces(), reference_interfaces_);

  // Read the robot state from the hardware
  state_reader_->read_state_interfaces(joint_space_state_);

  try
  {
    robot_kinematics_->compute_forward_kinematics(joint_space_state_.positions, joint_space_state_.velocities, end_effector_frame_,
                                                  motion_reference_frame_, task_space_state_);
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error computing forward kinematics: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  task_space_state_.motion_frame = motion_reference_frame_;
  task_space_state_.wrench_frame = force_torque_measure_frame_;

  // Initialize the task space reference to NaN when the controller is activated
  acg_message_utilities::clear(task_space_reference_);

  // Initialize the task space command with the current robot state when the controller is activated
  task_space_command_.pose = task_space_state_.pose;
  previous_task_space_command_.pose = task_space_state_.pose;

  if (command_writer_->has_task_space_twist_interface())
  {
    task_space_command_.twist = geometry_msgs::msg::Twist();
    previous_task_space_command_.twist = geometry_msgs::msg::Twist();
  }

  if (controller_state_ == State::FILTERS_INITIALIZATION)
  {
    RCLCPP_WARN(get_node()->get_logger(),
                "The controller is active in the filters initialization state. Waiting for the sensor bias filter to became reliable.");
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionBasedInteractionController::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */)
{
  state_reader_->release_interfaces();
  command_writer_->release_interfaces();
  reference_reader_->release_interfaces();
  force_torque_sensor_reader_->release_interfaces();
  release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionBasedInteractionController::on_error(const rclcpp_lifecycle::State& /* previous_state */)
{
  controller_state_ = State::ERROR_HANDLING;
  RCLCPP_ERROR(get_node()->get_logger(), "The controller is in a error state. The last valid command will be commanded.");
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> MotionBasedInteractionController::on_export_reference_interfaces()
{
  // Resize the reference_interfaces_ vector to the size of the reference interface names vector with NaN values
  ChainableControllerInterface::reference_interfaces_.resize(reference_reader_->available_interfaces().size(),
                                                             std::numeric_limits<double>::quiet_NaN());

  return reference_reader_->build_reference_interfaces(reference_interfaces_);
}

controller_interface::return_type MotionBasedInteractionController::update_reference_from_subscribers()
{
  return controller_interface::return_type::OK;
}

bool MotionBasedInteractionController::on_set_chained_mode(bool /* chained_mode */)
{
  return true;
}

controller_interface::return_type MotionBasedInteractionController::update_and_write_commands(const rclcpp::Time& time,
                                                                                              const rclcpp::Duration& period)
{
  // Read the robot state from the hardware
  state_reader_->read_state_interfaces(joint_space_state_);

  try
  {
    robot_kinematics_->compute_forward_kinematics(joint_space_state_.positions, joint_space_state_.velocities, end_effector_frame_,
                                                  motion_reference_frame_, task_space_state_);
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error computing forward kinematics: %s", e.what());
    controller_state_ = State::ERROR_HANDLING;
  }
  task_space_state_.motion_frame = motion_reference_frame_;

  // Read the force/torque sensor state from the hardware
  force_torque_sensor_reader_->read_state_interfaces(task_space_state_.wrench);
  task_space_state_.wrench_frame = force_torque_measure_frame_;

  switch (controller_state_)
  {
    case State::ERROR_HANDLING:
    {
      // Set the task space command to the last task space command
      set_safe_task_space_command_();
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), DURATION_MS_, "The controller is in a error state.");
    }
    break;
    case State::FILTERS_INITIALIZATION:
    {
      if (!filter_wrench_())
      {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Error filtering wrench. The controller is in a error state. The last valid command will be commanded.");
        controller_state_ = State::ERROR_HANDLING;
        break;
      }

      if (is_bias_computed_)
      {
        RCLCPP_INFO(get_node()->get_logger(), "The filters have been initialized, allowing for the use of the controller now.");
        controller_state_ = State::CONTROL_LAW;
      }

      // Set the task space command to the last task space command
      set_safe_task_space_command_();
    }
    break;
    case State::CONTROL_LAW:
    {
      // Read the task space reference from the reference interfaces
      reference_reader_->read_from_reference_interfaces(task_space_reference_);
      task_space_reference_.motion_frame = motion_reference_frame_;
      task_space_reference_.wrench_frame = wrench_reference_frame_;

      // Check for NaN values in the task space reference
      if ((reference_reader_->has_task_space_pose_interface() && acg_message_utilities::is_nan(task_space_reference_.pose)) ||
          (reference_reader_->has_task_space_twist_interface() && acg_message_utilities::is_nan(task_space_reference_.twist)) ||
          (reference_reader_->has_task_space_twist_interface() && acg_message_utilities::is_nan(task_space_reference_.wrench)) ||
          (reference_reader_->has_task_space_wrench_interface() && acg_message_utilities::is_nan(task_space_reference_.wrench)))
      {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), DURATION_MS_,
                             "Task space reference is NaN. Using the last valid reference.");

        // Set the task space command to the last task space command
        set_safe_task_space_command_();

        break;
      }

      // Filter the wrench using the filter chain
      if (!filter_wrench_())
      {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Error filtering wrench. The controller is in a error state. The last valid command will be commanded.");
        controller_state_ = State::ERROR_HANDLING;
        break;
      }

      if (!compute_control_law(time, period))
      {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Error computing control law. The controller is in a error state. The last valid command will be commanded.");
        controller_state_ = State::ERROR_HANDLING;
      }
    }
    break;
  }
  // Check if the task space command is close to the task space state
  if (!acg_kinematics::is_pose_close(task_space_command_.pose, task_space_state_.pose, parameters_.command_tolerance.translational_tolerance,
                                     parameters_.command_tolerance.rotational_tolerance))
  {
    RCLCPP_WARN(get_node()->get_logger(), "The task space command is not close to the task space state.");

    // If the task space command is not close to the task space state, set the task space command to the last valid command
    set_safe_task_space_command_();
    controller_state_ = State::ERROR_HANDLING;
  }

  // Write the command to the command interfaces
  command_writer_->write_to_command_interfaces(task_space_command_);

  previous_task_space_command_.pose = task_space_command_.pose;
  if (command_writer_->has_task_space_twist_interface())
  {
    previous_task_space_command_.twist = task_space_command_.twist;
  }

  return controller_interface::return_type::OK;
}

bool MotionBasedInteractionController::filter_wrench_()
{
  // Initialize the force filter chain input and output vectors
  std::vector<double> force_filter_chain_input(filter_chain_number_of_channels_, 0.0);
  std::vector<double> force_filter_chain_output(filter_chain_number_of_channels_, 0.0);

  // Fill the input vector with the force/torque sensor values
  force_filter_chain_input[0] = task_space_state_.wrench.force.x;
  force_filter_chain_input[1] = task_space_state_.wrench.force.y;
  force_filter_chain_input[2] = task_space_state_.wrench.force.z;
  force_filter_chain_input[3] = task_space_state_.wrench.torque.x;
  force_filter_chain_input[4] = task_space_state_.wrench.torque.y;
  force_filter_chain_input[5] = task_space_state_.wrench.torque.z;

  if (filter_chain_number_of_channels_ == GRAVITY_COMPENSATION_FILTER_CHANNELS)
  {
    // Read the force/torque sensor pose
    acg_control_msgs::msg::TaskSpacePoint ft_sensor_task_space_state;
    try
    {
      robot_kinematics_->compute_forward_kinematics(joint_space_state_.positions, joint_space_state_.velocities, motion_reference_frame_,
                                                    force_torque_measure_frame_, ft_sensor_task_space_state);
    }
    catch (std::runtime_error& e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Error computing forward kinematics: %s", e.what());
      return false;
    }

    force_filter_chain_input[6] = ft_sensor_task_space_state.pose.orientation.w;
    force_filter_chain_input[7] = ft_sensor_task_space_state.pose.orientation.x;
    force_filter_chain_input[8] = ft_sensor_task_space_state.pose.orientation.y;
    force_filter_chain_input[9] = ft_sensor_task_space_state.pose.orientation.z;
  }

  // Update the filter chain with the input vector
  if (!force_filter_chain_->update(force_filter_chain_input, force_filter_chain_output))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error updating force filter chain.");
    return false;
  }

  // Assign the filtered values to the wrench
  // The force/torque sensor values are negated to be the exerted ones
  task_space_state_.wrench.force.x = -force_filter_chain_output[0];
  task_space_state_.wrench.force.y = -force_filter_chain_output[1];
  task_space_state_.wrench.force.z = -force_filter_chain_output[2];
  task_space_state_.wrench.torque.x = -force_filter_chain_output[3];
  task_space_state_.wrench.torque.y = -force_filter_chain_output[4];
  task_space_state_.wrench.torque.z = -force_filter_chain_output[5];

  if (!is_bias_computed_ && force_filter_chain_output[filter_chain_number_of_channels_ - 1] != 0.0)
  {
    is_bias_computed_ = true;
  }

  return true;
}

void MotionBasedInteractionController::set_safe_task_space_command_()
{
  // Set the task space command to a safe state based on the last valid command
  task_space_command_.pose = previous_task_space_command_.pose;

  if (command_writer_->has_task_space_twist_interface())
  {
    task_space_command_.twist = geometry_msgs::msg::Twist();
  }
}
}  // namespace motion_based_interaction_controller
