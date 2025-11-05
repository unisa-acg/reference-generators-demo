/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   gazebo_ft_sensor.cpp
 * Author:  Alessio Coone
 * Org.:    UNISA
 * Date:    Sep 8, 2024
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include "gazebo_force_torque_sensor/gazebo_ft_sensor.hpp"
#include "gazebo_force_torque_sensor/exceptions.hpp"

namespace gz_ros2_control
{
GazeboFTSensor::GazeboFTSensor()
  : hw_sensor_states_(Vector6d::Zero())
  , hw_sensor_commands_(Eigen::Matrix<double, 1, 1>::Zero())
  , node_(std::make_shared<rclcpp::Node>("subscriber"))
  , number_of_samples_for_computing_bias_(0)
  , logger_(rclcpp::get_logger("gazebo_ft_sensor"))
  , sensor_and_gravity_voltage_bias_(Vector6d::Zero())
  , bias_computed_(false)
  , number_of_attempts_(0)
  , stable_readings_(0)
  , previous_reading_(Vector6d::Zero())

{}

GazeboFTSensor::~GazeboFTSensor() {}

hardware_interface::CallbackReturn GazeboFTSensor::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Store sensor informations
  sensor_ = info.sensors[0];

  // Check the number of declared state interfaces
  if (sensor_.state_interfaces.size() != FT_SENSOR_AXES_)
  {
    RCLCPP_ERROR(logger_, "The Gazebo driver expects %d state interfaces. Number of state interfaces specified: %zu", FT_SENSOR_AXES_,
                 sensor_.state_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GazeboFTSensor::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  set_number_of_samples_for_computing_bias_(std::stoi(info_.hardware_parameters["number_of_samples_for_computing_bias"]));
  set_stable_readings_tolerance_(std::stod(info_.hardware_parameters["stable_readings_tolerance"]));
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GazeboFTSensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  RCLCPP_INFO(logger_, "Exporting State Interfaces");
  RCLCPP_INFO(logger_, "State:");

  // Export the state interfaces
  for (std::size_t i = 0; i < sensor_.state_interfaces.size(); ++i)
  {
    RCLCPP_INFO(logger_, "\t%s", sensor_.state_interfaces[i].name.c_str());
    state_interfaces.emplace_back(sensor_.name, sensor_.state_interfaces[i].name, &hw_sensor_states_[i]);
  }

  return state_interfaces;
}

hardware_interface::CallbackReturn GazeboFTSensor::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  subscriber_ = node_->create_subscription<geometry_msgs::msg::Wrench>("/force_torque",                   // topic name
                                                                       rclcpp::QoS(rclcpp::KeepLast(1)),  // quality of service
                                                                       std::bind(&GazeboFTSensor::fts_callback_, this, std::placeholders::_1));

  // Initialize the hw_sensor_states_ variable to Nan
  for (auto& value : hw_sensor_states_)
  {
    value = std::numeric_limits<double>::quiet_NaN();
  }

  sensor_readings_.writeFromNonRT(hw_sensor_states_);

  node_thread_ = std::make_unique<std::thread>([&]() { executor_.add_node(node_); });

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GazeboFTSensor::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  executor_.cancel();
  subscriber_.reset();
  node_thread_->join();
  node_thread_.reset();
  node_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type GazeboFTSensor::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  Vector6d force_torque;
  get_measure_(force_torque);

  if (!bias_computed_ && number_of_samples_for_computing_bias_ > 0)
  {
    compute_bias_(force_torque);
  }

  hw_sensor_states_ = *(sensor_readings_.readFromRT());

  return hardware_interface::return_type::OK;
}

void GazeboFTSensor::fts_callback_(const geometry_msgs::msg::Wrench& msg)
{
  wrench_msg_ = msg;
}

void GazeboFTSensor::compute_bias_(const Vector6d force_torque)
{
  const unsigned int MAX_READS = number_of_samples_for_computing_bias_ + TOLERANCE_FOR_BIAS_CALCULATION_;

  // Check if at least one measure is beyond tolerance (non-zero); if so, this reading is a candidate stable reading
  bool non_zero_measure = !force_torque.isZero(1e-6);

  // To consider a measure as stable, all measures should be very similar to the previous ones. To achieve this,
  // we check if the current measure is close to the previous one. An alternative approach could be to use
  // Eigen::isApproxToConstant, but this would require setting a relative tolerance based on the measure's value,
  // whereas our tolerance is absolute.
  bool equal_to_previous = (force_torque - previous_reading_).isZero(stable_readings_tolerance_);

  stable_readings_ = (non_zero_measure && equal_to_previous) ? stable_readings_ + 1 : 0;

  previous_reading_ = force_torque;

  // Throw an exception if we did not reach the desired number of stable readings
  if (number_of_attempts_ > MAX_READS)
    throw gz_ros2_control::Exception("Cannot retrieve stable measures from the F/T sensor");

  if (stable_readings_ >= number_of_samples_for_computing_bias_)
  {
    bias_computed_ = true;
    RCLCPP_INFO(logger_, "Bias successfully computed");
    // Update the bias
    sensor_and_gravity_voltage_bias_ = force_torque;

    RCLCPP_DEBUG(logger_, "Force/torque sensor bias set to return null measures: force bias [x: %f, y: %f, z: %f], torque bias [x: %f, y: %f, z: %f]",
                 sensor_and_gravity_voltage_bias_[0], sensor_and_gravity_voltage_bias_[1], sensor_and_gravity_voltage_bias_[2],
                 sensor_and_gravity_voltage_bias_[3], sensor_and_gravity_voltage_bias_[4], sensor_and_gravity_voltage_bias_[5]);
  }

  number_of_attempts_++;
}

void GazeboFTSensor::get_measure_(Vector6d& force_torque)
{
  // Callback "fts_callback_" processing is asynchronous with this method.
  // If more than one message is received between two spin_some calls, all of them are discarded, but the last one.
  executor_.spin_some();
  tf2::fromMsg(wrench_msg_, force_torque);
  force_torque = force_torque - sensor_and_gravity_voltage_bias_;
  sensor_readings_.writeFromNonRT(force_torque);
}

std::vector<hardware_interface::CommandInterface> GazeboFTSensor::export_command_interfaces()
{
  return std::vector<hardware_interface::CommandInterface>();
}

hardware_interface::return_type GazeboFTSensor::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  return hardware_interface::return_type::OK;
}

bool GazeboFTSensor::initSim(rclcpp::Node::SharedPtr& /*model_nh*/, std::map<std::string, ignition::gazebo::Entity>& /*joints*/,
                             const hardware_interface::HardwareInfo& /*hardware_info*/, ignition::gazebo::EntityComponentManager& /*_ecm*/,
                             int& /*update_rate*/)
{
  return true;
}

void GazeboFTSensor::set_number_of_samples_for_computing_bias_(const int number_samples)
{
  if (number_samples == 0)
  {
    RCLCPP_WARN(logger_, "The number of samples for bias calculation is set to 0, so the bias will not be calculated.");
  }
  else if (number_samples < 0)
  {
    throw gz_ros2_control::Exception("Number of samples must be bigger than 0");
  }

  number_of_samples_for_computing_bias_ = number_samples;
}

void GazeboFTSensor::set_stable_readings_tolerance_(const double tolerance)
{
  if (tolerance < 0.0)
  {
    throw gz_ros2_control::Exception("Stable readings tolerance must be bigger than or equal to 0");
  }
  stable_readings_tolerance_ = tolerance;
}

}  // namespace gz_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(gz_ros2_control::GazeboFTSensor, gz_ros2_control::GazeboSimSystemInterface)
