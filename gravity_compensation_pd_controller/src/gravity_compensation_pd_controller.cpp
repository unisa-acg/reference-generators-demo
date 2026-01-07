/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   gravity_compensation_pd_controller.cpp
 * Author:  Davide Risi, Gerardo Ricciardelli
 * Org.:    UNISA
 * Date:    Dec 1, 2024
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <pluginlib/class_list_macros.hpp>

#include <xxx_common_libraries/urdf_utilities.hpp>

#include "gravity_compensation_pd_controller/gravity_compensation_pd_controller.hpp"

namespace gravity_compensation_pd_controller
{
GravityCompensationPDController::GravityCompensationPDController()
  : controller_interface::ChainableControllerInterface()
  , dynamics_solver_loader_(std::make_shared<pluginlib::ClassLoader<inverse_dynamics_solver::InverseDynamicsSolver>>(
        "inverse_dynamics_solver", "inverse_dynamics_solver::InverseDynamicsSolver"))
  , dynamics_solver_(nullptr)
{}

controller_interface::CallbackReturn GravityCompensationPDController::on_init()
{
  try
  {
    parameter_handler_ = std::make_shared<gravity_compensation_pd_controller::ParamListener>(get_node());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Number of joints to control is fixed after initialization
  num_joints_ = parameter_handler_->get_params().joints.size();

  // The inverse dynamics solver requires this node parameter interface to have the `robot_description`.
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

  // Retrieve torque limits from URDF
  std::vector<double> torque_limits;
  try
  {
    urdf_utilities::read_joint_effort_limits_from_urdf(robot_description, parameter_handler_->get_params().joints, torque_limits);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception while reading joint effort limits from URDF: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Copying local variable torque limits to the class member torque_limits_
  torque_limits_.resize(torque_limits.size());
  std::copy(torque_limits.begin(), torque_limits.end(), torque_limits_.data());

  // Allocate dynamic memory
  robot_joint_state_.positions.assign(num_joints_, 0.0);
  robot_joint_state_.velocities.assign(num_joints_, 0.0);
  joint_command_.effort.assign(num_joints_, 0.0);
  joint_reference_.positions.assign(num_joints_, 0.0);
  last_joint_reference_ = joint_reference_;

  // Allocate Eigen vectors' dynamic memory for real-time safeness
  position_error_.resize(num_joints_);
  eigen_effort_command_.resize(num_joints_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GravityCompensationPDController::command_interface_configuration() const
{
  return { controller_interface::interface_configuration_type::INDIVIDUAL, command_writer_.available_interfaces() };
}

controller_interface::InterfaceConfiguration GravityCompensationPDController::state_interface_configuration() const
{
  return { controller_interface::interface_configuration_type::INDIVIDUAL, state_reader_.available_state_interfaces() };
}

std::vector<hardware_interface::CommandInterface> GravityCompensationPDController::on_export_reference_interfaces()
{
  // Resize the reference_interfaces_ vector to the size of the reference interface names vector with NaN values
  ChainableControllerInterface::reference_interfaces_.resize(reference_reader_.available_interfaces().size(),
                                                             std::numeric_limits<double>::quiet_NaN());

  return reference_reader_.build_reference_interfaces(ChainableControllerInterface::reference_interfaces_);
}

controller_interface::CallbackReturn GravityCompensationPDController::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Set the command to NaN values, to notify the hardware that the controller is unable to provide valid commands
  std::fill(joint_command_.effort.begin(), joint_command_.effort.end(), std::numeric_limits<double>::quiet_NaN());
  command_writer_.write_to_command_interfaces(joint_command_);

  RCLCPP_ERROR(get_node()->get_logger(),
               "Controller is in error state. Writing NaN values to command interfaces. Restart the controller to recover.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationPDController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  gravity_compensation_pd_controller::Params params = parameter_handler_->get_params();
  xxx_hardware_interface_facade::StateInterfaceNamesOverrideConfig state_interfaces_names_override;
  state_interfaces_names_override.position_state_interfaces = params.state_interfaces_names_override.position;
  state_interfaces_names_override.velocity_state_interfaces = params.state_interfaces_names_override.velocity;

  const std::vector<std::string> STATE_INTERFACES{ "position", "velocity" };

  state_reader_.configure_state_interfaces(STATE_INTERFACES, params.joints, params.robot_name, state_interfaces_names_override);

  if (!state_reader_.has_joint_position_state_interface() || !state_reader_.has_joint_velocity_state_interface())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error configuring state interfaces, joint position and/or velocity state interfaces not found");
    return controller_interface::CallbackReturn::ERROR;
  }

  xxx_hardware_interface_facade::CommandInterfaceNamesOverrideConfig command_interfaces_names_override;
  command_interfaces_names_override.joint_effort_interface_names = params.command_interfaces_names_override.effort;
  const std::vector<std::string> COMMAND_INTERFACES{ "effort" };
  command_writer_.configure_interfaces(params.joints, COMMAND_INTERFACES, std::vector<std::string>(), params.robot_name, std::string(""),
                                       command_interfaces_names_override);

  if (!command_writer_.has_joint_effort_interface())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error configuring command interfaces, joint effort command interface not found");
    return controller_interface::CallbackReturn::ERROR;
  }

  xxx_hardware_interface_facade::CommandInterfaceNamesOverrideConfig reference_interfaces_names_override;
  reference_interfaces_names_override.joint_position_interface_names = params.reference_interfaces_names_override.joint_position;
  const std::vector<std::string> REFERENCE_INTERFACES{ "position" };
  reference_reader_.configure_interfaces(params.joints, REFERENCE_INTERFACES, std::vector<std::string>(), get_node()->get_name(), std::string(""),
                                         reference_interfaces_names_override);

  if (!reference_reader_.has_joint_position_interface())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error configuring reference interfaces, joint position reference interface not found");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (dynamics_solver_ == nullptr)
  {
    try
    {
      dynamics_solver_ = dynamics_solver_loader_->createSharedInstance(params.dynamics_solver.dynamics_solver_plugin);
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Exception while loading the dynamics solver plugin '%s': '%s'",
                   params.dynamics_solver.dynamics_solver_plugin.c_str(), ex.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  try
  {
    dynamics_solver_->initialize(get_node()->get_node_parameters_interface(), "dynamics_solver");
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception while initializing the dynamics solver plugin '%s': '%s'",
                 params.dynamics_solver.dynamics_solver_plugin.c_str(), e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params.p_gains.size() != num_joints_ || params.d_gains.size() != num_joints_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Gains size does not match joints size");
    return controller_interface::CallbackReturn::ERROR;
  }

  Kp_.diagonal() = Eigen::Map<const Eigen::VectorXd>(params.p_gains.data(), params.p_gains.size());
  Kd_.diagonal() = Eigen::Map<const Eigen::VectorXd>(params.d_gains.data(), params.d_gains.size());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationPDController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Assign the state interfaces provided to this controller to the internal state reader
  if (!state_reader_.assign_loaned_state_interfaces(ControllerInterfaceBase::state_interfaces_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error assigning the state interfaces to the state reader.");
    return controller_interface::CallbackReturn::ERROR;
  }

  command_writer_.assign_loaned_command_interfaces(ControllerInterfaceBase::command_interfaces_);
  reference_reader_.assign_command_interfaces(on_export_reference_interfaces(), ChainableControllerInterface::reference_interfaces_);

  // Read the current state of the robot from the hardware
  state_reader_.read_state_interfaces(robot_joint_state_);

  if (std::any_of(robot_joint_state_.positions.begin(), robot_joint_state_.positions.end(), [](double pos) { return std::isnan(pos); }))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to read joint positions from the hardware.\n");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Use current joint state as first valid reference
  joint_reference_.positions = robot_joint_state_.positions;
  last_joint_reference_ = joint_reference_;
  std::fill(joint_command_.effort.begin(), joint_command_.effort.end(), std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GravityCompensationPDController::update_reference_from_subscribers()
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type GravityCompensationPDController::update_and_write_commands(const rclcpp::Time& /*time*/,
                                                                                             const rclcpp::Duration& /*period*/)
{
  // Read the current state of the robot from the hardware
  state_reader_.read_state_interfaces(robot_joint_state_);
  if (std::any_of(robot_joint_state_.positions.begin(), robot_joint_state_.positions.end(), [](double pos) { return std::isnan(pos); }))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to read joint positions from the hardware.\n");
    return controller_interface::return_type::ERROR;
  }

  // Read the reference from the reference interfaces
  reference_reader_.read_from_reference_interfaces(joint_reference_);

  if (std::any_of(joint_reference_.positions.begin(), joint_reference_.positions.end(), [](double pos) { return std::isnan(pos); }))
  {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), DURATION_MS_,
                         "Received NaN values in joint reference positions. Using last valid reference instead.");
    // If NaN values are detected in the joint reference positions, use the last valid reference
    joint_reference_ = last_joint_reference_;
  }

  compute_control_law_();

  // Write the command to the hardware
  command_writer_.write_to_command_interfaces(joint_command_);

  // Update the last command and reference
  last_joint_reference_ = joint_reference_;

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn GravityCompensationPDController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  state_reader_.release_interfaces();
  command_writer_.release_interfaces();
  reference_reader_.release_interfaces();
  release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool GravityCompensationPDController::on_set_chained_mode(bool /* chained_mode */)
{
  return true;
}

void GravityCompensationPDController::compute_control_law_()
{
  Eigen::VectorXd current_positions = Eigen::Map<const Eigen::VectorXd>(robot_joint_state_.positions.data(), robot_joint_state_.positions.size());
  Eigen::VectorXd current_velocities = Eigen::Map<const Eigen::VectorXd>(robot_joint_state_.velocities.data(), robot_joint_state_.velocities.size());

  for (std::size_t i = 0; i < num_joints_; i++)
  {
    position_error_(i) = joint_reference_.positions[i] - current_positions(i);
  }

  // Compute the effort command using PD control law
  if (parameter_handler_->get_params().compensate_gravity)
  {
    eigen_effort_command_ = dynamics_solver_->getGravityVector(current_positions);
  }
  else
  {
    eigen_effort_command_.setZero();
  }

  eigen_effort_command_ += Kp_ * position_error_ - Kd_ * current_velocities;

  // Apply torque limits using Eigen
  eigen_effort_command_ = eigen_effort_command_.cwiseMax(-torque_limits_).cwiseMin(torque_limits_);
  std::copy(eigen_effort_command_.data(), eigen_effort_command_.data() + num_joints_, joint_command_.effort.begin());
}

}  // namespace gravity_compensation_pd_controller

PLUGINLIB_EXPORT_CLASS(gravity_compensation_pd_controller::GravityCompensationPDController, controller_interface::ChainableControllerInterface)
