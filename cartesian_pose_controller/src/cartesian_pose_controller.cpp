/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   cartesian_pose_controller.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Apr 7, 2025
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <pluginlib/class_list_macros.hpp>
#include <acg_common_libraries/message_utilities.hpp>
#include <acg_common_libraries/urdf_utilities.hpp>

#include "cartesian_pose_controller/cartesian_pose_controller.hpp"

namespace cartesian_pose_controller
{

controller_interface::CallbackReturn CartesianPoseController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    parameter_handler_ = std::make_shared<cartesian_pose_controller::ParamListener>(get_node());
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "Exception thrown during init stage with message: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // Declare parameter alpha for the kinematics interface
  if (!get_node()->has_parameter("alpha"))
  {
    get_node()->declare_parameter("alpha", parameter_handler_->get_params().alpha);
  }

  // Number of joints to control is fixed after initialization
  num_joints_ = parameter_handler_->get_params().joints.size();

  // The kinematics_interface requires the node to have the `robot_description` parameter.
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

  // Initializing internal variables from the configuration file
  task_space_reference_frame_ = parameter_handler_->get_params().root_link_frame;
  tip_link_ = parameter_handler_->get_params().kinematics.tip;

  if (robot_description.find(task_space_reference_frame_) == std::string::npos)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "The task space reference frame '%s' is not in the robot description.",
                 task_space_reference_frame_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (robot_description.find(tip_link_) == std::string::npos)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "The tip link '%s' is not in the robot description.", tip_link_.c_str());
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

  // Assign and initialize the internal variables that can vary according to the robot's configuration
  robot_joint_state_.positions.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());

  robot_kinematics_.initialize(num_joints_, kinematics_);
  eigen_joint_command_velocities_.resize(num_joints_);

  // Initialize the K matrix with the gains from the parameter handler
  K_matrix_.setIdentity();
  for (std::size_t i = 0; i < acg_kinematics::NUM_CARTESIAN_DOF; i++)
  {
    K_matrix_(i, i) = parameter_handler_->get_params().k_matrix_gains[i];
  }

  // Read the joint position limits from the URDF
  try
  {
    urdf_utilities::read_joint_position_limits_from_urdf(robot_description, parameter_handler_->get_params().joints, joint_positions_upper_limits_,
                                                         joint_positions_lower_limits_);
    urdf_utilities::read_joint_velocity_limits_from_urdf(robot_description, parameter_handler_->get_params().joints, joint_velocity_limits_);
  }
  catch (const urdf_utilities::JointLimitsNotDefinedException& e)
  {
    RCLCPP_WARN(get_node()->get_logger(), "Joint limits not defined in URDF: %s", e.what());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown while reading joint position limits from URDF: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseController::on_configure(const rclcpp_lifecycle::State&)
{
  // Configure the state interfaces based on the configuration file. Note that the position state interface is always required.
  std::vector<std::string> state_interfaces_names;
  state_interfaces_names.emplace_back("position");

  acg_hardware_interface_facade::StateInterfaceNamesOverrideConfig state_interfaces_names_override;
  state_interfaces_names_override.position_state_interfaces = parameter_handler_->get_params().state_interfaces_names_override.position;

  state_reader_.configure_state_interfaces(state_interfaces_names, parameter_handler_->get_params().joints,
                                           parameter_handler_->get_params().robot_name, state_interfaces_names_override);

  // Configure the command interfaces based on the configuration file
  std::vector<std::string> command_interface_names = parameter_handler_->get_params().command_interfaces;

  // Setting the command interface names override struct based on the configuration file for the command interfaces
  acg_hardware_interface_facade::CommandInterfaceNamesOverrideConfig command_interface_names_override;
  command_interface_names_override.joint_position_interface_names = parameter_handler_->get_params().command_interfaces_names_override.joint_position;
  command_interface_names_override.joint_velocity_interface_names = parameter_handler_->get_params().command_interfaces_names_override.joint_velocity;

  command_writer_.configure_interfaces(parameter_handler_->get_params().joints, command_interface_names, std::vector<std::string>(),
                                       parameter_handler_->get_params().joint_space_command_controller, "", command_interface_names_override);

  // Configure the reference interfaces based on the configuration file
  std::vector<std::string> reference_interfaces;
  reference_interfaces.emplace_back("pose");

  // Setting the command interface names override struct based on the configuration file for the reference interfaces
  acg_hardware_interface_facade::CommandInterfaceNamesOverrideConfig reference_interface_names_override;
  reference_interface_names_override.task_space_pose_interface_names =
      parameter_handler_->get_params().reference_interfaces_names_override.task_space_pose;
  if (parameter_handler_->get_params().use_twist_reference)
  {
    reference_interfaces.emplace_back("twist");
    reference_interface_names_override.task_space_twist_interface_names =
        parameter_handler_->get_params().reference_interfaces_names_override.task_space_twist;
  }

  reference_reader_.configure_interfaces(std::vector<std::string>(), std::vector<std::string>(), reference_interfaces, std::string(),
                                         get_node()->get_name(), reference_interface_names_override);

  // Even if position commands are not used, it is required to store them because the control law needs to compute the robot pose
  joint_space_command_.positions.assign(num_joints_, 0.0);
  last_command_.positions.assign(num_joints_, 0.0);

  if (command_writer_.has_joint_velocity_interface())
  {
    joint_space_command_.velocities.assign(num_joints_, 0.0);
    last_command_.velocities.assign(num_joints_, 0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseController::on_activate(const rclcpp_lifecycle::State&)
{
  // Assign the state interfaces provided to this controller to the internal state reader
  if (!state_reader_.assign_loaned_state_interfaces(ControllerInterfaceBase::state_interfaces_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error assigning the state interfaces to the state reader.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Read the current position of the joints from the robot
  state_reader_.read_state_interfaces(robot_joint_state_);

  // Log in the debug level the current joint state on the console
  RCLCPP_DEBUG(get_node()->get_logger(), "Joint position during activation:");
  for (std::size_t i = 0; i < robot_joint_state_.positions.size(); i++)
  {
    RCLCPP_DEBUG(get_node()->get_logger(), "Joint %ld: %f", i, robot_joint_state_.positions[i]);
  }

  reference_reader_.assign_command_interfaces(on_export_reference_interfaces(), reference_interfaces_);
  command_writer_.assign_loaned_command_interfaces(ControllerInterfaceBase::command_interfaces_);

  // Initialize the last command with the current joint state
  joint_space_command_.positions = robot_joint_state_.positions;
  last_command_.positions = robot_joint_state_.positions;

  robot_kinematics_.compute_forward_kinematics(robot_joint_state_.positions, tip_link_, task_space_reference_frame_, last_reference_.pose);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseController::on_deactivate(const rclcpp_lifecycle::State&)
{
  ControllerInterfaceBase::release_interfaces();
  reference_reader_.release_interfaces();
  command_writer_.release_interfaces();
  state_reader_.release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CartesianPoseController::command_interface_configuration() const
{
  // Configure the command interface for the joint space reference
  if (parameter_handler_->get_params().joint_space_command_controller.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No command controller specified in the configuration file.");
    return { controller_interface::interface_configuration_type::NONE, {} };
  }

  return { controller_interface::interface_configuration_type::INDIVIDUAL, command_writer_.available_interfaces() };
}

controller_interface::InterfaceConfiguration CartesianPoseController::state_interface_configuration() const
{
  // Configure the state interface for the joint space reference
  const std::vector<std::string> state_interface_names{ state_reader_.available_state_interfaces() };
  if (state_interface_names.size() == 0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "State interfaces required are not available.");
    return { controller_interface::interface_configuration_type::NONE, {} };
  }

  return { controller_interface::interface_configuration_type::INDIVIDUAL, state_interface_names };
}

std::vector<hardware_interface::CommandInterface> CartesianPoseController::on_export_reference_interfaces()
{
  // Resize the reference_interfaces_ vector to the size of the reference interface names vector with NaN values
  ChainableControllerInterface::reference_interfaces_.resize(reference_reader_.available_interfaces().size(),
                                                             std::numeric_limits<double>::quiet_NaN());

  return reference_reader_.build_reference_interfaces(reference_interfaces_);
}

controller_interface::return_type CartesianPoseController::update_reference_from_subscribers()
{
  return controller_interface::return_type::OK;
}

bool CartesianPoseController::on_set_chained_mode(bool)
{
  return true;
}

controller_interface::return_type CartesianPoseController::update_and_write_commands(const rclcpp::Time& /* time */, const rclcpp::Duration& period)
{
  reference_reader_.read_from_reference_interfaces(task_space_reference_);
  state_reader_.read_state_interfaces(robot_joint_state_);

  switch (controller_state_)
  {
    case State::ERROR_HANDLING:
    {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), DURATION_MS_, "The controller is in a error state.");
    }
    break;
    case State::CONTROL_LAW:
    {
      // Check if the task space reference is valid.
      // Note that the task space reference is not valid (contains NaN values) when the previous controller in the chain is not activated.
      if (acg_message_utilities::is_nan(task_space_reference_.pose) || acg_message_utilities::is_nan(task_space_reference_.twist))
      {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), DURATION_MS_,
                             "Task space reference is NaN. Using the last valid reference.");

        // Setting a safe joint command to the robot
        set_safe_joint_command_();

        break;
      }

      if (!compute_control_law_(period))
      {
        RCLCPP_FATAL(get_node()->get_logger(), "Error in computing control law");
        controller_state_ = State::ERROR_HANDLING;
      }

      if (!is_joint_command_within_tolerance_())
      {
        set_safe_joint_command_();

        RCLCPP_FATAL(get_node()->get_logger(), "The command is too far from the current joint state. The controller is in an error state.");
        controller_state_ = State::ERROR_HANDLING;
      }
      break;
    }
    default:
      RCLCPP_FATAL(get_node()->get_logger(), "Unknown controller state.");
      return controller_interface::return_type::ERROR;
  }

  // Write the joint space command to the command interfaces
  command_writer_.write_to_command_interfaces(joint_space_command_);

  last_command_ = joint_space_command_;
  return controller_interface::return_type::OK;
}

bool CartesianPoseController::compute_control_law_(const rclcpp::Duration& period)
{
  geometry_msgs::msg::Pose task_space_robot_state;
  robot_kinematics_.compute_forward_kinematics(joint_space_command_.positions, tip_link_, task_space_reference_frame_, task_space_robot_state);

  Vector6d pose_error;
  acg_kinematics::compute_pose_error(task_space_reference_.pose, task_space_robot_state, pose_error);

  Vector6d desired_twist;
  if (reference_reader_.has_task_space_twist_interface())
  {
    // The desired twist is set to the task space twist reference and converted to eigen vector
    tf2::fromMsg(task_space_reference_.twist, desired_twist);
  }
  else
  {
    // Computing the desired twist from the last reference and the current reference
    Vector6d reference_diff;
    acg_kinematics::compute_pose_error(task_space_reference_.pose, last_reference_.pose, reference_diff);
    desired_twist = reference_diff / period.seconds();
  }

  // Convert the joint space command positions to an Eigen vector
  const Eigen::VectorXd eigen_joint_command_positions =
      Eigen::Map<const Eigen::VectorXd>(joint_space_command_.positions.data(), joint_space_command_.positions.size());

  // Computing the joint velocities through the damped least squares inverse Jacobian
  kinematics_->convert_cartesian_deltas_to_joint_deltas(eigen_joint_command_positions, K_matrix_ * pose_error + desired_twist, tip_link_,
                                                        eigen_joint_command_velocities_);

  // If the controller writes the velocities to the command interfaces, values are clamped to the joint velocity limits
  if (command_writer_.has_joint_velocity_interface())
  {
    // Convert the joint velocities from Eigen vector to std::vector
    std::copy(eigen_joint_command_velocities_.data(), eigen_joint_command_velocities_.data() + eigen_joint_command_velocities_.size(),
              joint_space_command_.velocities.begin());

    // Clamp the joint velocities to the joint limits
    for (std::size_t i = 0; i < num_joints_; i++)
    {
      joint_space_command_.velocities[i] = std::clamp(joint_space_command_.velocities[i], -joint_velocity_limits_[i], joint_velocity_limits_[i]);
    }
  }

  // Integrate the joint velocities to get the new joint space command and clamp the values to the joint limits
  for (std::size_t i = 0; i < num_joints_; i++)
  {
    // Integrate the joint velocities to get the new joint space command
    joint_space_command_.positions[i] = last_command_.positions[i] + (eigen_joint_command_velocities_[i] * period.seconds());

    // Clamp the joint space command to the joint limits
    joint_space_command_.positions[i] =
        std::clamp(joint_space_command_.positions[i], joint_positions_lower_limits_[i], joint_positions_upper_limits_[i]);
  }

  // Saving the last reference for the next iteration
  last_reference_ = task_space_reference_;
  return true;
};

bool CartesianPoseController::is_joint_command_within_tolerance_()
{
  // Check if the command is close to the current joint state
  if (command_writer_.has_joint_position_interface())
  {
    for (std::size_t i = 0; i < num_joints_; i++)
    {
      if (std::abs(joint_space_command_.positions[i] - robot_joint_state_.positions[i]) > parameter_handler_->get_params().joint_position_tolerance)
      {
        return false;
      }
    }
  }
  if (command_writer_.has_joint_velocity_interface())
  {
    if (std::any_of(joint_space_command_.velocities.begin(), joint_space_command_.velocities.end(),
                    [this](double velocity) { return std::abs(velocity) > parameter_handler_->get_params().joint_velocity_tolerance; }))
    {
      return false;
    }
  }
  return true;
}

void CartesianPoseController::set_safe_joint_command_()
{
  // Set the joint space command to the last command
  if (command_writer_.has_joint_position_interface())
  {
    joint_space_command_.positions = last_command_.positions;
  }
  if (command_writer_.has_joint_velocity_interface())
  {
    std::fill(joint_space_command_.velocities.begin(), joint_space_command_.velocities.end(), 0.0);
  }
}

}  // namespace cartesian_pose_controller

PLUGINLIB_EXPORT_CLASS(cartesian_pose_controller::CartesianPoseController, controller_interface::ChainableControllerInterface)
