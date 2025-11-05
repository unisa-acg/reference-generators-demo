/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   gravity_compensation_filter.cpp
 * Author:  Alessio Coone, Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Oct 7, 2024
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <eigen3/Eigen/Dense>
#include <urdf/model.h>
#include <chrono>
#include <acg_common_libraries/urdf_utilities.hpp>
#include "gravity_compensation_filter/gravity_compensation_filter.hpp"

using namespace urdf_utilities;

namespace gravity_compensation_filter
{

template <typename T>
GravityCompensationFilter<T>::GravityCompensationFilter() : logger_(rclcpp::get_logger("gravity_compensation_filter"))
{}

template <typename T>
GravityCompensationFilter<T>::~GravityCompensationFilter()
{}

template <typename T>
bool GravityCompensationFilter<T>::configure()
{
  // Get the robot description
  rclcpp::NodeOptions node_options;
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("parameter_client_node", node_options);
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/robot_state_publisher");

  while (!parameters_client->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_INFO(logger_, "Service not available, waiting again...");
  }

  std::string robot_description = parameters_client->get_parameter<std::string>("robot_description", "");

  if (robot_description.empty())
  {
    RCLCPP_ERROR(logger_, "The robot description is empty");
    return false;
  }

  std::string ft_sensing_frame_name;
  if (!params_interface_->has_parameter(param_prefix_ + "ft_sensing_frame_name"))
  {
    RCLCPP_ERROR(logger_, "Could not retrieve parameter 'ft_sensing_frame_name' from the node's parameters");
    return false;
  }
  ft_sensing_frame_name = params_interface_->get_parameter(param_prefix_ + "ft_sensing_frame_name").as_string();

  if (ft_sensing_frame_name.empty())
  {
    RCLCPP_ERROR(logger_, "The 'ft_sensing_frame_name' parameter is empty");
    return false;
  }

  std::vector<double> gravity;
  if (!params_interface_->has_parameter(param_prefix_ + "gravity"))
  {
    gravity = { 0.0, 0.0, -9.81 };
    RCLCPP_INFO(logger_, "Could not retrieve parameter 'gravity' from the node's parameters. Using default value: [%f, %f, %f]", gravity[0],
                gravity[1], gravity[2]);
  }
  else
  {
    gravity = params_interface_->get_parameter(param_prefix_ + "gravity").as_double_array();

    if (gravity.size() != 3)
    {
      RCLCPP_ERROR(logger_, "The gravity vector must have 3 components");
      return false;
    }
    RCLCPP_INFO(logger_, "Using gravity vector: [%f, %f, %f]", gravity[0], gravity[1], gravity[2]);
  }

  // Load the URDF model
  urdf::Model robot_urdf;
  if (!robot_urdf.initString(robot_description))
  {
    RCLCPP_ERROR(logger_, "Failed to parse the robot description");
    return false;
  }

  // Get the force/torque sensing frame link
  urdf::LinkConstSharedPtr ft_sensing_frame_link = robot_urdf.getLink(ft_sensing_frame_name);
  if (ft_sensing_frame_link == nullptr)
  {
    RCLCPP_ERROR(logger_, "'%s' link not found in URDF", ft_sensing_frame_name.c_str());
    return false;
  }

  // Compute the mass and COM of the payload chain with respect to the force/torque sensing frame
  std::tuple<double, Eigen::Vector3d> payload_mass_com = urdf_utilities::compute_chain_mass_and_com(ft_sensing_frame_link);
  double payload_mass = std::get<0>(payload_mass_com);
  payload_com_ = std::get<1>(payload_mass_com);

  RCLCPP_INFO(logger_, "Gravity compensation assuming %f kg as payload mass and [%f, %f, %f] as payload COM", payload_mass, payload_com_(0),
              payload_com_(1), payload_com_(2));

  // Compute the payload force in base frame
  payload_force_ = Eigen::Vector3d(gravity.data()) * payload_mass;

  return true;
}

template <typename T>
bool GravityCompensationFilter<T>::update(const std::vector<T>& data_in, std::vector<T>& data_out)
{
  if (!configured_)
  {
    RCLCPP_ERROR(logger_, "The filter must be configured before calling the update() method.");
    return false;
  }

  if (data_in.size() != number_of_channels_)
  {
    RCLCPP_ERROR(logger_, "Input data size is %lu, while the expected size is %lu.", data_in.size(), number_of_channels_);
    return false;
  }

  // Copy the input data to the output data to avoid modifying the unused channels
  data_out = data_in;

  Eigen::Quaternion<T> ft_sensor_to_base_rotation(data_in[FT_SENSOR_AXES], data_in[FT_SENSOR_AXES + 1], data_in[FT_SENSOR_AXES + 2],
                                                  data_in[FT_SENSOR_AXES + 3]);

  // Compute the payload force in the force/torque sensor frame
  Eigen::Vector3d payload_force_in_ft_sensor_frame = ft_sensor_to_base_rotation.matrix() * payload_force_;

  // Compute the payload torque in the force/torque sensor frame
  Eigen::Vector3d payload_torque_in_ft_sensor_frame = payload_com_.cross(payload_force_in_ft_sensor_frame);

  // Compose the payload wrench in the force/torque sensor frame
  Eigen::Matrix<T, 6, 1> payload_wrench_in_ft_sensor_frame;
  payload_wrench_in_ft_sensor_frame << payload_force_in_ft_sensor_frame, payload_torque_in_ft_sensor_frame;

  // Remove gravity bias from the force/torque sensor data
  for (size_t i = 0; i < FT_SENSOR_AXES; i++)
  {
    data_out[i] -= payload_wrench_in_ft_sensor_frame(i);
  }

  return true;
}
}  // namespace gravity_compensation_filter

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gravity_compensation_filter::GravityCompensationFilter<double>, filters::MultiChannelFilterBase<double>)
