/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   gazebo_ft_sensor.hpp
 * Author:  Alessio Coone
 * Org.:    UNISA
 * Date:    Sep 8, 2024
 *
 * This class contains an implementation of GazeboSimSystemInterface.
 * It accesses the F/T sensor data via ROS2 topics, optionally
 * removes the bias, and writes the resulting measurements to the
 * state interfaces.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <memory>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <eigen3/Eigen/Core>
#include <gz_ros2_control/gz_system_interface.hpp>
#include "acg_common_libraries/message_utilities.hpp"

namespace gz_ros2_control
{

class GazeboFTSensor : public gz_ros2_control::GazeboSimSystemInterface
{
  /**
   * @brief A type alias for a 6x1 Eigen matrix of doubles.
   */
  typedef Eigen::Matrix<double, 6, 1> Vector6d;

public:
  /**
   * @brief Construct a GazeboFTSensor object.
   */
  GazeboFTSensor();

  /**
   * @brief Destroy a GazeboFTSensor object.
   */
  ~GazeboFTSensor();

  /**
   * @brief See the documentation in the base class: hardware_interface::SystemInterface::on_init().
   */
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  /**
   * @brief See the documentation in the base class: hardware_interface::SystemInterface::export_state_interfaces().
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief See the documentation in the base class: hardware_interface::SystemInterface::on_configure().
   */
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override;

  /**
   * @brief See the documentation in the base class: hardware_interface::SystemInterface::on_activate().
   */
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override;

  /**
   * @brief See the documentation in the base class: hardware_interface::SystemInterface::on_deactivate().
   */
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override;

  /**
   * @brief See the documentation in the base class: hardware_interface::SystemInterface::read().
   */
  hardware_interface::return_type read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override;

  /**
   * @brief See the documentation in the base class: hardware_interface::SystemInterface::export_command_interfaces().
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief See the documentation in the base class: hardware_interface::SystemInterface::write().
   */
  hardware_interface::return_type write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override;

  /**
   * @brief See the documentation in the base class: gz_ros2_control::GazeboSimSystemInterface::initSim().
   */
  bool initSim(rclcpp::Node::SharedPtr& model_nh, std::map<std::string, ignition::gazebo::Entity>& joints,
               const hardware_interface::HardwareInfo& hardware_info, ignition::gazebo::EntityComponentManager& _ecm, int& update_rate) override;

private:
  /**
   * @brief A callback called each time a message is published on the topic /force_torque.
   */
  void fts_callback_(const geometry_msgs::msg::Wrench& msg);

  /**
   * @brief Read unbiased force/torque measures from the sensor.
   *
   * This method can be used whenever the robot works with fixed orientation, otherwise, the bias must be recomputed.
   *
   * @param[out] force_torque unbiased force/torque measure.
   */
  void get_measure_(Vector6d& force_torque);

  /**
   * @brief Set the number of samples to average to estimate the bias.
   *
   * @param[in] number_samples number of samples used in the average.
   *
   * @throw force_torque_sensor::Exception if number_samples is zero.
   */
  void set_number_of_samples_for_computing_bias_(const int number_samples);

  /**
   * @brief Set the tolerance used to determine if the sensor readings are stable enough to compute the bias.
   *
   * @param[in] tolerance The tolerance value.
   *
   * @throw force_torque_sensor::Exception if tolerance is zero or negative.
   */
  void set_stable_readings_tolerance_(const double tolerance);

  /**
   * @brief Compute measure bias by reading a defined number of samples from the sensor.
   * The bias is then subtracted to the reading each time a new measure is acquired.
   *
   * @throw gz_ros2_control::Exception if the number of samples to read is zero.
   */
  void compute_bias_(const Vector6d force_torque);

  static const unsigned short FT_SENSOR_AXES_ = 6;
  static const unsigned short TOLERANCE_FOR_BIAS_CALCULATION_ = 10;
  static const unsigned short DEFAULT_UPDATE_RATE_ = 100;

  /**
   * @brief The tolerance used to determine if the sensor readings are stable enough to compute the bias.
   * If the difference between the current reading and the previous one is less than this value, the reading is considered stable.
   */
  double stable_readings_tolerance_ = 1e-6;

  /**
   * @brief The force/torque measurements to be written on the state interfaces.
   */
  Vector6d hw_sensor_states_;

  /**
   * @brief The command values to be written.
   */
  Eigen::Matrix<double, 1, 1> hw_sensor_commands_;

  /**
   * @brief Object that stores information about components defined for a specific hardware
   *  in robot's URDF.
   */
  hardware_interface::ComponentInfo sensor_;

  /**
   * @brief Pointer to a thread running the ROS2 node.
   */
  std::unique_ptr<std::thread> node_thread_;

  /**
   * @brief Shared pointer to the ROS2 node instance.
   */
  std::shared_ptr<rclcpp::Node> node_;

  /**
   * @brief The number of samples to read to compute the voltage bias.
   */
  int number_of_samples_for_computing_bias_;

  /**
   * @brief Subscriber that allows to capture force/torque published on the topic.
   */
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr subscriber_;

  /**
   * @brief Logger instance for the GazeboFTSensor class.
   */
  rclcpp::Logger logger_;

  /**
   * @brief Single-threaded executor for handling ROS2 callbacks.
   */
  rclcpp::executors::SingleThreadedExecutor executor_;

  /**
   * @brief A real-time buffer for storing sensor measurements.
   */
  realtime_tools::RealtimeBuffer<Vector6d> sensor_readings_;

  /**
   * @brief The force/torque measurement published on the topic by the simulated sensor.
   */
  geometry_msgs::msg::Wrench wrench_msg_;

  /**
   * @brief The voltage bias containing also the gravity contribution subtracted to the reading each time a new
   * measure is acquired.
   */
  Vector6d sensor_and_gravity_voltage_bias_;

  /**
   * @brief Flag indicating whether the sensor bias has been computed.
   */
  bool bias_computed_;

  /**
   * @brief The number of attempts made to collect stable sensor readings.
   */
  unsigned int number_of_attempts_;

  /**
   * @brief The number of consecutive stable readings obtained from the sensor.
   */
  int stable_readings_;

  /**
   * @brief The previous force/torque reading.
   */
  Vector6d previous_reading_;
};

}  // namespace gz_ros2_control
