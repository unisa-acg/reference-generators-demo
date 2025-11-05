/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   motion_based_interaction_controller.hpp
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Feb 4, 2025
 *
 * This module defines an abstract class for motion-based interaction
 * controllers.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <memory>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <controller_interface/chainable_controller_interface.hpp>
#include <filters/filter_chain.hpp>
#include <kinematics_interface/kinematics_interface.hpp>

#include <acg_control_msgs/msg/task_space_point.hpp>
#include <acg_common_libraries/kinematics.hpp>
#include <acg_hardware_interface_facade/command_writer.hpp>
#include <acg_hardware_interface_facade/reference_reader.hpp>
#include <acg_hardware_interface_facade/state_reader.hpp>
#include <acg_hardware_interface_facade/force_torque_sensor_reader.hpp>
#include <interaction_filter_base/interaction_filter_base.hpp>

#include "motion_based_interaction_controller/motion_based_interaction_controller_parameters.hpp"
#include "motion_based_interaction_controller/visibility_control.h"

namespace motion_based_interaction_controller
{
/**
 * @brief Enum defining the states of a motion-based interaction controller
 */
enum State
{
  FILTERS_INITIALIZATION,
  CONTROL_LAW,
  ERROR_HANDLING,
};

/**
 * @class MotionBasedInteractionController
 * @brief Abstract class for the motion-based interaction controllers.
 *
 * This class implements the \c controller_interface::ChainableControllerInterface interface and defines the interface for the motion-based
 * interaction controllers. All of the public methods override the corresponding methods of the \c controller_interface::ChainableControllerInterface
 * class.
 */
class MotionBasedInteractionController : public controller_interface::ChainableControllerInterface
{
public:
  /**
   * @brief Number of axes of the force/torque sensor.
   */
  static const unsigned int FT_SENSOR_AXES = 6;

  /**
   * @brief Size of the quaternion representing the orientation.
   */
  static const unsigned int ORIENTATION_SIZE = 4;

  /**
   * @brief Number of channels of the gravity compensation filter.
   */
  static const unsigned int GRAVITY_COMPENSATION_FILTER_CHANNELS = FT_SENSOR_AXES + ORIENTATION_SIZE + 1;

  MOTION_BASED_INTERACTION_CONTROLLER_PUBLIC
  MotionBasedInteractionController() = default;

  MOTION_BASED_INTERACTION_CONTROLLER_PUBLIC
  virtual ~MotionBasedInteractionController() = default;

  /**
   * @brief Refer to the superclass documentation.
   */
  MOTION_BASED_INTERACTION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  /**
   * @brief Refer to the superclass documentation.
   */
  MOTION_BASED_INTERACTION_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief Refer to the superclass documentation.
   */
  MOTION_BASED_INTERACTION_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @brief Refer to the superclass documentation.
   */
  MOTION_BASED_INTERACTION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& /* previous_state */) override;

  /**
   * @brief Refer to the superclass documentation.
   */
  MOTION_BASED_INTERACTION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& /* previous_state */) override;

  /**
   * @brief Refer to the superclass documentation.
   */
  MOTION_BASED_INTERACTION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) override;

  /**
   * @brief Refer to the superclass documentation.
   */
  MOTION_BASED_INTERACTION_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& /* previous_state */) override;

protected:
  /**
   * @brief Refer to the superclass documentation.
   */
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  /**
   * @brief Refer to the superclass documentation.
   */
  controller_interface::return_type update_reference_from_subscribers() override;

  /**
   * @brief Refer to the superclass documentation.
   */
  bool on_set_chained_mode(bool /* chained_mode */) override;

  /**
   * @brief Refer to the superclass documentation.
   */
  controller_interface::return_type update_and_write_commands(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /**
   * @brief Compute the control law for the motion-based interaction controller.
   *
   * This method must set the \c task_space_command_ member variable to the desired command.
   * If the controller enters the error state, the control law is not computed, and the last valid command is sent to the robot.
   *
   * @param[in] time The current time.
   * @param[in] period The period of the control loop.
   * @return True if the control law was computed successfully, false otherwise.
   */
  virtual bool compute_control_law(const rclcpp::Time& time, const rclcpp::Duration& period) = 0;

  /**
   * @brief Parameter handler to handle the controller parameters.
   */
  std::shared_ptr<motion_based_interaction_controller::ParamListener> parameter_handler_;

  /**
   * @brief Controller parameters.
   */
  motion_based_interaction_controller::Params parameters_;

  /**
   * @brief Task space command that the controller sends to the robot.
   */
  acg_control_msgs::msg::TaskSpacePoint task_space_command_;

  /**
   * @brief Previous task space command that the controller has sent to the robot.
   */
  acg_control_msgs::msg::TaskSpacePoint previous_task_space_command_;

  /**
   * @brief Task space reference that the controller should track.
   */
  acg_control_msgs::msg::TaskSpacePoint task_space_reference_;

  /**
   * @brief Robot state in the task space.
   */
  acg_control_msgs::msg::TaskSpacePoint task_space_state_;

  /**
   * @brief Robot state in the joint space.
   */
  acg_hardware_interface_facade::RobotJointState joint_space_state_;

  /**
   * @brief Robot command writer.
   */
  std::shared_ptr<acg_hardware_interface_facade::CommandWriter> command_writer_;

  /**
   * @brief Robot state reader.
   */
  std::shared_ptr<acg_hardware_interface_facade::StateReader> state_reader_;

  /**
   * @brief Controller reference reader.
   */
  std::shared_ptr<acg_hardware_interface_facade::ReferenceReader> reference_reader_;

  /**
   * @brief Force/torque sensor reader.
   */
  std::shared_ptr<acg_hardware_interface_facade::ForceTorqueSensorReader> force_torque_sensor_reader_;

  /**
   * @brief The name of the frame in which the task space reference, state, and command are expressed.
   */
  std::string motion_reference_frame_;

  /**
   * @brief The name of the frame in which the reference wrench is expressed.
   */
  std::string wrench_reference_frame_;

  /**
   * @brief The name of the end-effector frame.
   */
  std::string end_effector_frame_;

  /**
   * @brief The name of the frame in which the force/torque measures are expressed.
   */
  std::string force_torque_measure_frame_;

  /**
   * @brief The number of the robot joints.
   */
  std::size_t number_of_joints_;

  /**
   * @brief Shared pointer to the kinematics interface loader, responsible for loading the kinematics interface plugin.
   */
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> kinematics_loader_;

  /**
   * @brief Unique pointer to the kinematics interface, responsible for handling the kinematics of the robot.
   */
  std::shared_ptr<kinematics_interface::KinematicsInterface> kinematics_;

  /**
   * @brief Kinematics utilities object.
   */
  std::shared_ptr<acg_kinematics::RTKinematicsSolver> robot_kinematics_;

  /**
   * @brief Filter chain for the force/torque measurements.
   */
  std::shared_ptr<filters::MultiChannelFilterChain<double>> force_filter_chain_;

  /**
   * @brief The number of channels in the filter chain.
   */
  std::size_t filter_chain_number_of_channels_;

  /**
   * @brief Flag indicating whether the sensor bias has been computed.
   */
  bool is_bias_computed_{ false };

  /**
   * @brief Shared pointer to the interaction filter loader, responsible for loading the interaction filter plugin.
   */
  std::shared_ptr<pluginlib::ClassLoader<interaction_filters::InteractionFilterBase>> interaction_filter_loader_;

  /**
   * @brief Shared pointer to the interaction filter.
   */
  std::shared_ptr<interaction_filters::InteractionFilterBase> interaction_filter_;

  /**
   * @brief Constant to store the duration of the throttle interval as an integral value in milliseconds.
   */
  static const unsigned short int DURATION_MS_{ 1000 };

private:
  /**
   * @brief Filter the exerted wrench using the filter chain.
   *
   * Filtering also involves removing sensor bias, which requires an initial computation phase.
   * During this phase, wrenches are filtered anyway, but they may not be consistent.
   * Once the bias is computed, the \c is_bias_computed_ flag is set to true, and the controller can
   * transition to the control law state.
   *
   * @return True if the wrench was filtered successfully, false otherwise.
   */
  bool filter_wrench_();

  /**
   * @brief Set the task space command to a safe state based on the last valid command.
   *
   * A state is considered safe when avoids invalid or unstable poses, ensuring the robot remains within operational limits.
   * It corresponds to the current state of the robot, i.e. the last state state reached.
   */
  void set_safe_task_space_command_();

  /**
   * @brief The current internal state of the controller.
   */
  State controller_state_;

};  // class MotionBasedInteractionController

}  // namespace motion_based_interaction_controller
