/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   twist_sensor.hpp
 * Author:  Salvatore Paolino
 * Org.:    UNISA
 * Date:    May 13, 2025
 *
 * This class implements a twist sensor semantic component.
 *
 * -------------------------------------------------------------------
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <semantic_components/semantic_component_interface.hpp>

namespace xxx_semantic_components
{
/**
 * @class TwistSensor
 * @brief A semantic component that represents a twist sensor.
 *
 * This class provides an interface for managing the state of a twist sensor,
 * which includes linear and angular velocities in three dimensions. For further
 * details on the implemented interface, refer to the [SemanticComponentInterface
 * documentation](https://github.com/ros-controls/ros2_control/blob/humble/controller_interface/include/semantic_components/semantic_component_interface.hpp).
 */
class TwistSensor : public semantic_components::SemanticComponentInterface<geometry_msgs::msg::Twist>
{
public:
  static constexpr unsigned short int TWIST_SIZE{ 6 };

  TwistSensor(const std::string& name, const std::vector<std::string>& interface_names = {}) : SemanticComponentInterface(name, TWIST_SIZE)
  {
    if (!interface_names.empty())
    {
      if (interface_names.size() != TWIST_SIZE)
      {
        throw std::invalid_argument("TwistSensor: interface_names size (" + std::to_string(interface_names.size()) + ") must be equal to " +
                                    std::to_string(TWIST_SIZE));
      }
      interface_names_ = interface_names;
    }
    else
    {
      // Use standard interface names
      interface_names_.emplace_back(name_ + "/linear_velocity.x");
      interface_names_.emplace_back(name_ + "/linear_velocity.y");
      interface_names_.emplace_back(name_ + "/linear_velocity.z");
      interface_names_.emplace_back(name_ + "/angular_velocity.x");
      interface_names_.emplace_back(name_ + "/angular_velocity.y");
      interface_names_.emplace_back(name_ + "/angular_velocity.z");
    }
  }

  ~TwistSensor() = default;

  std::vector<std::string> get_state_interface_names() const
  {
    return interface_names_;
  }

  bool get_values_as_message(geometry_msgs::msg::Twist& twist) const
  {
    if (state_interfaces_.size() != TWIST_SIZE)
    {
      return false;
    }

    twist.linear.x = state_interfaces_[0].get().get_value();
    twist.linear.y = state_interfaces_[1].get().get_value();
    twist.linear.z = state_interfaces_[2].get().get_value();
    twist.angular.x = state_interfaces_[3].get().get_value();
    twist.angular.y = state_interfaces_[4].get().get_value();
    twist.angular.z = state_interfaces_[5].get().get_value();

    return true;
  }
};
}  // namespace xxx_semantic_components
