/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   message_utilities.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 7, 2025
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <rclcpp/rclcpp.hpp>

#include "xxx_common_libraries/message_utilities.hpp"

namespace xxx_message_utilities
{

void log_info(const xxx_control_msgs::msg::TaskSpacePoint& task_space_point)
{
  RCLCPP_INFO(
      rclcpp::get_logger("message_utilities"),
      "Task space reference: \n\tposition: %f %f %f \n\torientation: %f %f %f %f \n\tlinear_twist: %f %f %f \n\tangular_twist: %f %f %f \n\t"
      "linear_acceleration: %f %f %f \n\tangular_acceleration: %f %f %f \n\tforce: %f %f %f \n\ttorque: %f %f %f \n\tforce_derivative: %f %f %f "
      "\n\ttorque_derivative: %f %f %f \n\tmotion_frame: %s \n\twrench_frame: %s",
      task_space_point.pose.position.x, task_space_point.pose.position.y, task_space_point.pose.position.z, task_space_point.pose.orientation.x,
      task_space_point.pose.orientation.y, task_space_point.pose.orientation.z, task_space_point.pose.orientation.w, task_space_point.twist.linear.x,
      task_space_point.twist.linear.y, task_space_point.twist.linear.z, task_space_point.twist.angular.x, task_space_point.twist.angular.y,
      task_space_point.twist.angular.z, task_space_point.acceleration.linear.x, task_space_point.acceleration.linear.y,
      task_space_point.acceleration.linear.z, task_space_point.acceleration.angular.x, task_space_point.acceleration.angular.y,
      task_space_point.acceleration.angular.z, task_space_point.wrench.force.x, task_space_point.wrench.force.y, task_space_point.wrench.force.z,
      task_space_point.wrench.torque.x, task_space_point.wrench.torque.y, task_space_point.wrench.torque.z,
      task_space_point.wrench_derivative.force.x, task_space_point.wrench_derivative.force.y, task_space_point.wrench_derivative.force.z,
      task_space_point.wrench_derivative.torque.x, task_space_point.wrench_derivative.torque.y, task_space_point.wrench_derivative.torque.z,
      task_space_point.motion_frame.c_str(), task_space_point.wrench_frame.c_str());
}

void clear(geometry_msgs::msg::Wrench& wrench)
{
  wrench.force.x = std::numeric_limits<double>::quiet_NaN();
  wrench.force.y = std::numeric_limits<double>::quiet_NaN();
  wrench.force.z = std::numeric_limits<double>::quiet_NaN();
  wrench.torque.x = std::numeric_limits<double>::quiet_NaN();
  wrench.torque.y = std::numeric_limits<double>::quiet_NaN();
  wrench.torque.z = std::numeric_limits<double>::quiet_NaN();
}

void clear(xxx_control_msgs::msg::TaskSpacePoint& task_space_point)
{
  task_space_point.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  task_space_point.pose.position.y = std::numeric_limits<double>::quiet_NaN();
  task_space_point.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  task_space_point.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
  task_space_point.pose.orientation.y = std::numeric_limits<double>::quiet_NaN();
  task_space_point.pose.orientation.z = std::numeric_limits<double>::quiet_NaN();
  task_space_point.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  task_space_point.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  task_space_point.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
  task_space_point.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  task_space_point.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  task_space_point.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  task_space_point.twist.angular.z = std::numeric_limits<double>::quiet_NaN();
  task_space_point.acceleration.linear.x = std::numeric_limits<double>::quiet_NaN();
  task_space_point.acceleration.linear.y = std::numeric_limits<double>::quiet_NaN();
  task_space_point.acceleration.linear.z = std::numeric_limits<double>::quiet_NaN();
  task_space_point.acceleration.angular.x = std::numeric_limits<double>::quiet_NaN();
  task_space_point.acceleration.angular.y = std::numeric_limits<double>::quiet_NaN();
  task_space_point.acceleration.angular.z = std::numeric_limits<double>::quiet_NaN();
  clear(task_space_point.wrench);
  clear(task_space_point.wrench_derivative);
  task_space_point.wrench_frame = "";
}

void clear(xxx_control_msgs::msg::JointWrenchPoint& joint_space_point)
{
  std::fill(joint_space_point.positions.begin(), joint_space_point.positions.end(), std::numeric_limits<double>::quiet_NaN());
  std::fill(joint_space_point.velocities.begin(), joint_space_point.velocities.end(), std::numeric_limits<double>::quiet_NaN());
  std::fill(joint_space_point.accelerations.begin(), joint_space_point.accelerations.end(), std::numeric_limits<double>::quiet_NaN());
  std::fill(joint_space_point.effort.begin(), joint_space_point.effort.end(), std::numeric_limits<double>::quiet_NaN());

  clear(joint_space_point.wrench);
}

geometry_msgs::msg::PoseStamped build_pose_stamped_msg(const geometry_msgs::msg::Pose& pose, const std::string& frame_id, const rclcpp::Time& time)
{
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = time;
  pose_stamped.header.frame_id = frame_id;
  pose_stamped.pose = pose;

  return pose_stamped;
}

bool is_nan(const geometry_msgs::msg::Pose& pose)
{
  return (std::isnan(pose.position.x) || std::isnan(pose.position.y) || std::isnan(pose.position.z) || std::isnan(pose.orientation.x) ||
          std::isnan(pose.orientation.y) || std::isnan(pose.orientation.z) || std::isnan(pose.orientation.w));
}

bool is_nan(const geometry_msgs::msg::Twist& twist)
{
  return (std::isnan(twist.linear.x) || std::isnan(twist.linear.y) || std::isnan(twist.linear.z) || std::isnan(twist.angular.x) ||
          std::isnan(twist.angular.y) || std::isnan(twist.angular.z));
}

bool is_nan(const geometry_msgs::msg::Wrench& wrench)
{
  return (std::isnan(wrench.force.x) || std::isnan(wrench.force.y) || std::isnan(wrench.force.z) || std::isnan(wrench.torque.x) ||
          std::isnan(wrench.torque.y) || std::isnan(wrench.torque.z));
}

bool is_valid(const geometry_msgs::msg::Twist& twist)
{
  return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) && std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
         std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}

}  // namespace xxx_message_utilities
