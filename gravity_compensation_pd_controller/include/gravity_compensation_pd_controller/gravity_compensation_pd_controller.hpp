/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   gravity_compensation_pd_controller.hpp
 * Author:  Davide Risi, Gerardo Ricciardelli
 * Org.:    UNISA
 * Date:    Dec 1, 2024
 *
 * This class implements a PD controller with gravity compensation.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <Eigen/Core>

#include <rclcpp/duration.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <pluginlib/class_loader.hpp>
#include <controller_interface/chainable_controller_interface.hpp>
#include <inverse_dynamics_solver/inverse_dynamics_solver.hpp>

#include <acg_hardware_interface_facade/state_reader.hpp>
#include <acg_hardware_interface_facade/reference_reader.hpp>
#include <acg_hardware_interface_facade/command_writer.hpp>

#include "gravity_compensation_pd_controller/gravity_compensation_pd_controller_parameters.hpp"
#include "gravity_compensation_pd_controller/visibility_control.h"

namespace gravity_compensation_pd_controller
{

/**
 * @class GravityCompensationPDController
 * @brief A class representing a PD controller with gravity compensation.
 *
 * This class implements the \c controller_interface::ChainableControllerInterface interface.
 * All of the public methods override the corresponding methods of the \c controller_interface::ChainableControllerInterface class.
 * Please refer to the documentation of the base class for more details.
 */
class GravityCompensationPDController : public controller_interface::ChainableControllerInterface
{
public:
  GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_PUBLIC
  GravityCompensationPDController();

  GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  GRAVITY_COMPENSATION_PD_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

protected:
  // The following methods are overridden from the base class. Refer to the base class documentation for details.
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  controller_interface::return_type update_reference_from_subscribers() override;
  bool on_set_chained_mode(bool chained_mode) override;
  controller_interface::return_type update_and_write_commands(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /**
   * @brief Computes the joint effort command using a PD control law with optional gravity compensation.
   *
   * This method calculates the joint effort command according to the control law described in the README file.
   * If \c compensate_gravity parameter is disabled, the control law reduces to a standard PD controller.
   * The resulting effort command is stored in the \c joint_command_ attribute.
   */
  void compute_control_law_();

  /**
   * @brief Plugin loader for the inverse dynamics solver.
   */
  std::shared_ptr<pluginlib::ClassLoader<inverse_dynamics_solver::InverseDynamicsSolver>> dynamics_solver_loader_;

  /**
   * @brief Shared pointer to the inverse dynamics solver used for gravity compensation.
   */
  std::shared_ptr<inverse_dynamics_solver::InverseDynamicsSolver> dynamics_solver_;

  /**
   * @brief Variables to store the joint command, reference, and last reference.
   */
  acg_control_msgs::msg::JointWrenchPoint joint_command_, joint_reference_, last_joint_reference_;

  /**
   * @brief Shared pointer to the parameter listener responsible for handling the controller's parameters.
   */
  std::shared_ptr<gravity_compensation_pd_controller::ParamListener> parameter_handler_;

  /**
   * @brief Vector to store the torque limits for each joint.
   */
  Eigen::VectorXd torque_limits_;

  /**
   * @brief Eigen vector to store the position error between the reference and current joint positions.
   */
  Eigen::VectorXd position_error_;

  /**
   * @brief Eigen vector to store the computed joint effort command.
   */
  Eigen::VectorXd eigen_effort_command_;

  /**
   * @brief Diagonal matrix for proportional gains (Kp) used in the PD control law.
   */
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> Kp_;

  /**
   * @brief Diagonal matrix for derivative gains (Kd) used in the PD control law.
   */
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> Kd_;

  /**
   * @brief Constant to store the duration of the throttle interval as an integral value in milliseconds.
   */
  static constexpr unsigned short DURATION_MS_{ 1000 };

  /**
   * @brief Number of joints to control.
   */
  std::size_t num_joints_{ 0 };

  /**
   * @brief Internal variable to store the current joint state of the robot.
   */
  acg_hardware_interface_facade::RobotJointState robot_joint_state_;

  /**
   * @brief Class for reading the reference input for the controller.
   */
  acg_hardware_interface_facade::ReferenceReader reference_reader_;

  /**
   * @brief Class for reading the state interfaces of the robot.
   */
  acg_hardware_interface_facade::StateReader state_reader_;

  /**
   * @brief Class for writing commands to the robot actuators.
   */
  acg_hardware_interface_facade::CommandWriter command_writer_;
};

}  // namespace gravity_compensation_pd_controller
