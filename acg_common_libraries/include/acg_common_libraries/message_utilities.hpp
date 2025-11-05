/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   message_utilities.hpp
 * Author:  Davide Risi, Alessio Coone
 * Org.:    UNISA
 * Date:    Feb 7, 2025
 *
 * This library contains utility functions for converting messages.
 *
 * -------------------------------------------------------------------
 */

#pragma once

// Eigen
#include <eigen3/Eigen/Core>

// ROS
#include <rclcpp/time.hpp>

// geometry_msgs
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// POINTS
#include <acg_control_msgs/msg/task_space_point.hpp>
#include <acg_control_msgs/msg/joint_wrench_point.hpp>

namespace tf2
{

/**
 * @brief Convert a \c geometry_msgs::msg::Wrench object to a \c{Eigen::Matrix<T, 6, 1>} object.
 *
 * This function transforms the force and torque measurements from a
 * \c geometry_msgs::msg::Wrench message into an \c{Eigen::Matrix<T, 6, 1>} format for further processing.
 *
 * @param[in] in The \c geometry_msgs::msg::Wrench object representing the force and torque measurements.
 * @param[out] out The resulting \c{Eigen::Matrix<T, 6, 1>} object containing the converted force and torque values.
 */
template <typename T>
inline void fromMsg(const geometry_msgs::msg::Wrench& in, Eigen::Matrix<T, 6, 1>& out);

/**
 * @brief Convert a \c geometry_msgs::msg::Pose object to a \c{Eigen::Matrix<T, 6, 1>} object.
 *
 * This function transforms the pose from a
 * \c geometry_msgs::msg::Pose message into an \c{Eigen::Matrix<T, 6, 1>} format for further processing.
 * The output vector is interpreted as a 6D pose. The first three components (in(0) to in(2)) represent
 * the position (x, y, z), while the last three components (in(3) to in(5)) represent
 * the orientation in Roll-Pitch-Yaw (RPY) angles. The RPY angles are computed from the quaternion using the
 * fixed-axis convention, as in \c tf2::Matrix3x3::getRPY. That is, the angles represent successive rotations
 * about the fixed X, Y, and Z axes, respectively.
 *
 * @param[in] in The \c geometry_msgs::msg::Pose object representing the pose.
 * @param[out] out The resulting \c{Eigen::Matrix<T, 6, 1>} object containing the converted pose values.
 */
template <typename T>
inline void fromMsg(const geometry_msgs::msg::Pose& in, Eigen::Matrix<T, 6, 1>& out);

/**
 * @brief Convert a \c geometry_msgs::msg::Twist object to a \c{Eigen::Matrix<T, 6, 1>} object.
 *
 * This function transforms the twist from a
 * \c geometry_msgs::msg::Twist message into an \c{Eigen::Matrix<T, 6, 1>} format for further processing.
 *
 * @param[in] in The \c geometry_msgs::msg::Twist object representing the twist.
 * @param[out] out The resulting \c{Eigen::Matrix<T, 6, 1>} object containing the converted twist values.
 */
template <typename T>
inline void fromMsg(const geometry_msgs::msg::Twist& in, Eigen::Matrix<T, 6, 1>& out);

/**
 * @brief Convert a \c geometry_msgs::msg::Accel object to a \c{Eigen::Matrix<T, 6, 1>} object.
 *
 * This function transforms the acceleration from a
 * \c geometry_msgs::msg::Accel message into an \c{Eigen::Matrix<T, 6, 1>} format for further processing.
 *
 * @param[in] in The \c geometry_msgs::msg::Accel object representing the acceleration.
 * @param[out] out The resulting \c{Eigen::Matrix<T, 6, 1>} object containing the converted acceleration values.
 */
template <typename T>
inline void fromMsg(const geometry_msgs::msg::Accel& in, Eigen::Matrix<T, 6, 1>& out);

/**
 * @brief Convert a \c{Eigen::Matrix<T, 6, 1>} vector to a \c geometry_msgs::msg::Wrench message.
 *
 * @param[in] in The \c{Eigen::Matrix<T, 6, 1>} vector to convert.
 * @param[out] out The resulting \c geometry_msgs::msg::Wrench message.
 */
template <typename T>
inline void toMsg(const Eigen::Matrix<T, 6, 1>& in, geometry_msgs::msg::Wrench& out);

/**
 * @brief Convert a \c{Eigen::Matrix<T, 6, 1>} vector to a \c geometry_msgs::msg::Pose message.
 * The input vector is interpreted as a 6D pose. The first three components (in(0) to in(2)) represent
 * the position (x, y, z), while the last three components (in(3) to in(5)) represent
 * the orientation in Roll-Pitch-Yaw (RPY) angles. The RPY angles are interpreted using the fixed-axis convention,
 * following the behavior of \c tf2::Quaternion::setRPY, i.e., rotations are applied in the order RPY
 * around fixed axes XYZ.
 *
 * @param[in] in The \c{Eigen::Matrix<T, 6, 1>} vector to convert.
 * @param[out] out The resulting \c geometry_msgs::msg::Pose message.
 */
template <typename T>
inline void toMsg(const Eigen::Matrix<T, 6, 1>& in, geometry_msgs::msg::Pose& out);

/**
 * @brief Convert a \c{Eigen::Matrix<T, 6, 1>} vector to a \c geometry_msgs::msg::Twist message.
 *
 * @param[in] in The \c{Eigen::Matrix<T, 6, 1>} vector to convert.
 * @param[out] out The resulting \c geometry_msgs::msg::Twist message.
 */
template <typename T>
inline void toMsg(const Eigen::Matrix<T, 6, 1>& in, geometry_msgs::msg::Twist& out);

/**
 * @brief Convert a \c{Eigen::Matrix<T, 6, 1>} vector to a \c geometry_msgs::msg::Accel message.
 *
 * @param[in] in The \c{Eigen::Matrix<T, 6, 1>} vector to convert.
 * @param[out] out The resulting \c geometry_msgs::msg::Accel message.
 */
template <typename T>
inline void toMsg(const Eigen::Matrix<T, 6, 1>& in, geometry_msgs::msg::Accel& out);

}  // namespace tf2

// ****** Definitions of tf2 namespace functions ******

template <typename T>
inline void tf2::fromMsg(const geometry_msgs::msg::Wrench& in, Eigen::Matrix<T, 6, 1>& out)
{
  out << in.force.x, in.force.y, in.force.z, in.torque.x, in.torque.y, in.torque.z;
}

template <typename T>
inline void tf2::fromMsg(const geometry_msgs::msg::Pose& in, Eigen::Matrix<T, 6, 1>& out)
{
  // Convert quaternion to Euler angles
  tf2::Quaternion tf2_quat(in.orientation.x, in.orientation.y, in.orientation.z, in.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);
  out << in.position.x, in.position.y, in.position.z, roll, pitch, yaw;
}

template <typename T>
inline void tf2::fromMsg(const geometry_msgs::msg::Twist& in, Eigen::Matrix<T, 6, 1>& out)
{
  out << in.linear.x, in.linear.y, in.linear.z, in.angular.x, in.angular.y, in.angular.z;
}

template <typename T>
inline void tf2::fromMsg(const geometry_msgs::msg::Accel& in, Eigen::Matrix<T, 6, 1>& out)
{
  out << in.linear.x, in.linear.y, in.linear.z, in.angular.x, in.angular.y, in.angular.z;
}

template <typename T>
inline void tf2::toMsg(const Eigen::Matrix<T, 6, 1>& in, geometry_msgs::msg::Wrench& out)
{
  out.force.x = in(0);
  out.force.y = in(1);
  out.force.z = in(2);
  out.torque.x = in(3);
  out.torque.y = in(4);
  out.torque.z = in(5);
}

template <typename T>
inline void tf2::toMsg(const Eigen::Matrix<T, 6, 1>& in, geometry_msgs::msg::Pose& out)
{
  out.position.x = in(0);
  out.position.y = in(1);
  out.position.z = in(2);

  tf2::Quaternion q;
  q.setRPY(in(3), in(4), in(5));
  out.orientation.x = q.x();
  out.orientation.y = q.y();
  out.orientation.z = q.z();
  out.orientation.w = q.w();
}

template <typename T>
inline void tf2::toMsg(const Eigen::Matrix<T, 6, 1>& in, geometry_msgs::msg::Twist& out)
{
  out.linear.x = in(0);
  out.linear.y = in(1);
  out.linear.z = in(2);
  out.angular.x = in(3);
  out.angular.y = in(4);
  out.angular.z = in(5);
}

template <typename T>
inline void tf2::toMsg(const Eigen::Matrix<T, 6, 1>& in, geometry_msgs::msg::Accel& out)
{
  out.linear.x = in(0);
  out.linear.y = in(1);
  out.linear.z = in(2);
  out.angular.x = in(3);
  out.angular.y = in(4);
  out.angular.z = in(5);
}

// ****** End of definitions of tf2 namespace functions ******

namespace acg_message_utilities
{

/**
 * @brief Log the task space point to the console in the \c INFO level.
 *
 * @param[in] task_space_point The task space point to log.
 */
void log_info(const acg_control_msgs::msg::TaskSpacePoint& task_space_point);

/**
 * @brief Set the fields of the wrench to NaN.
 *
 * @param[in,out] wrench The wrench to set to NaN.
 */
void clear(geometry_msgs::msg::Wrench& wrench);

/**
 * @brief Set all the fields of a task space point to NaN.
 *
 * @param[in,out] task_space_point The task space point to clear.
 */
void clear(acg_control_msgs::msg::TaskSpacePoint& task_space_point);

/**
 * @brief Set all the fields of a joint wrench point to NaN.
 *
 * @param[in,out] joint_space_point The joint wrench point to clear.
 */
void clear(acg_control_msgs::msg::JointWrenchPoint& joint_space_point);

/**
 * @brief Build a \c geometry_msgs::msg::PoseStamped message from a \c geometry_msgs::msg::Pose message.
 *
 * @param[in] pose The \c geometry_msgs::msg::Pose message to convert.
 * @param[in] frame_id The frame ID to set in the \c geometry_msgs::msg::PoseStamped message.
 * @param[in] time The time to set in the \c geometry_msgs::msg::PoseStamped message.
 */
geometry_msgs::msg::PoseStamped build_pose_stamped_msg(const geometry_msgs::msg::Pose& pose, const std::string& frame_id, const rclcpp::Time& time);

/**
 * @brief Check if the pose is NaN. A pose is considered NaN if any of its position or orientation components are NaN.
 *
 * @param[in] pose The pose to be checked.
 * @return true if any component of the pose is NaN, false otherwise.
 */
bool is_nan(const geometry_msgs::msg::Pose& pose);

/**
 * @brief Check if the twist is NaN. A twist is considered NaN if any of its linear or angular components are NaN.
 *
 * @param[in] twist The twist to be checked.
 * @return true if any component of the twist is NaN, false otherwise.
 */
bool is_nan(const geometry_msgs::msg::Twist& twist);

/**
 * @brief Check if the wrench is NaN. A wrench is considered NaN if any of its force or torque components are NaN.
 *
 * @param[in] wrench The wrench to check.
 * @return true if the wrench is NaN, false otherwise.
 */
bool is_nan(const geometry_msgs::msg::Wrench& wrench);

/**
 * @brief Validates the twist message.
 *
 * @param[in] twist The twist message to validate.
 * @return True if the twist is valid, false otherwise.
 */
bool is_valid(const geometry_msgs::msg::Twist& twist);

}  // namespace acg_message_utilities
