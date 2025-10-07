/*
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   pose_command.hpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 17, 2025
 *
 * This class implements a pose command semantic component.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "acg_semantic_components/semantic_component_command_interface.hpp"

namespace acg_semantic_components
{

class PoseCommand : public semantic_components::SemanticComponentCommandInterface<geometry_msgs::msg::Pose>
{
private:
  std::vector<std::string> generate_interface_names_(const std::string& name, const std::vector<std::string>& interface_names)
  {
    if (!interface_names.empty())
    {
      if (interface_names.size() != POSE_SIZE)
      {
        throw std::invalid_argument("PoseCommand: interface_names size (" + std::to_string(interface_names.size()) + ") must be equal to " +
                                    std::to_string(POSE_SIZE));
      }
      return interface_names;
    }
    else
    {
      // Use standard interface names
      return { { name + "/position.x" },    { name + "/position.y" },    { name + "/position.z" },   { name + "/orientation.x" },
               { name + "/orientation.y" }, { name + "/orientation.z" }, { name + "/orientation.w" } };
    }
  }

public:
  static constexpr unsigned short int POSE_SIZE{ 7 };

  PoseCommand(const std::string& name, const std::vector<std::string>& interface_names = {})
    : SemanticComponentCommandInterface(name, generate_interface_names_(name, interface_names))
  {}

  ~PoseCommand() = default;

  bool set_values_from_message(const geometry_msgs::msg::Pose& pose) override
  {
    if (command_interfaces_.size() != POSE_SIZE)
    {
      return false;
    }

    command_interfaces_[0].get().set_value(pose.position.x);
    command_interfaces_[1].get().set_value(pose.position.y);
    command_interfaces_[2].get().set_value(pose.position.z);
    command_interfaces_[3].get().set_value(pose.orientation.x);
    command_interfaces_[4].get().set_value(pose.orientation.y);
    command_interfaces_[5].get().set_value(pose.orientation.z);
    command_interfaces_[6].get().set_value(pose.orientation.w);

    return true;
  }

  bool get_values_as_message(geometry_msgs::msg::Pose& pose) override
  {
    if (command_interfaces_.size() != POSE_SIZE)
    {
      return false;
    }

    pose.position.x = command_interfaces_[0].get().get_value();
    pose.position.y = command_interfaces_[1].get().get_value();
    pose.position.z = command_interfaces_[2].get().get_value();
    pose.orientation.x = command_interfaces_[3].get().get_value();
    pose.orientation.y = command_interfaces_[4].get().get_value();
    pose.orientation.z = command_interfaces_[5].get().get_value();
    pose.orientation.w = command_interfaces_[6].get().get_value();

    return true;
  }
};

}  // namespace acg_semantic_components
