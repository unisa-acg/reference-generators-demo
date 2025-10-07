/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   kinematics.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 10, 2025
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include "acg_common_libraries/message_utilities.hpp"
#include "acg_common_libraries/kinematics.hpp"

namespace acg_kinematics
{

void RTKinematicsSolver::initialize(const std::size_t num_joints, const std::shared_ptr<kinematics_interface::KinematicsInterface>& kinematics)
{
  num_joints_ = num_joints;
  kinematics_ = kinematics;

  // Initialize the jacobian
  jacobian_.resize(NUM_CARTESIAN_DOF, num_joints_);

  is_initialized_ = true;
}

void RTKinematicsSolver::compute_forward_kinematics(const std::vector<double>& joint_positions, const std::string& end_effectors_frame,
                                                    const std::string& desired_base_frame, geometry_msgs::msg::Pose& pose)
{
  throw_error_if_not_initialized_();

  compute_desired_base_to_end_effector_transform_(joint_positions, end_effectors_frame, desired_base_frame);

  // Set the task space pose
  pose = tf2::toMsg(desired_base_to_end_effector_transform_);
}

void RTKinematicsSolver::compute_forward_kinematics(const std::vector<double>& joint_positions, const std::vector<double>& joint_velocities,
                                                    const std::string& end_effectors_frame, const std::string& desired_base_frame,
                                                    acg_control_msgs::msg::TaskSpacePoint& task_space_point)
{
  // Computing the forward kinematics of the robot's pose.
  // The function has the side effect of populating desired_base_to_end_effector_transform_
  compute_forward_kinematics(joint_positions, end_effectors_frame, desired_base_frame, task_space_point.pose);

  if (joint_velocities.size() != num_joints_)
  {
    throw std::runtime_error("The size of joint_velocities (" + std::to_string(joint_velocities.size()) +
                             ") is different from the number of joints (" + std::to_string(num_joints_) + ").");
  }

  // Compute task space twist only if the joint velocities are available
  if (std::none_of(joint_velocities.begin(), joint_velocities.end(), [](double value) { return std::isnan(value); }))
  {
    // Compute Jacobian
    kinematics_->calculate_jacobian(joint_positions, end_effectors_frame, jacobian_);

    // Convert joint velocities to Eigen vector
    // Note: Eigen::Map is used to avoid copying the data and allocating new memory
    const Eigen::VectorXd eigen_joint_velocities = Eigen::Map<const Eigen::VectorXd>(joint_velocities.data(), joint_velocities.size());

    // Compute rotated twist
    const Eigen::Matrix<double, NUM_CARTESIAN_DOF, 1> twist = jacobian_ * eigen_joint_velocities;
    tf2::toMsg(desired_base_to_end_effector_transform_.rotation() * twist.head(3), task_space_point.twist.linear);
    tf2::toMsg(desired_base_to_end_effector_transform_.rotation() * twist.tail(3), task_space_point.twist.angular);
  }
  else
  {
    throw std::runtime_error("The joint velocities are NaN. The twist cannot be computed.");
  }
}

void RTKinematicsSolver::compute_desired_base_to_end_effector_transform_(const std::vector<double>& joint_positions,
                                                                         const std::string& end_effectors_frame,
                                                                         const std::string& desired_base_frame)
{
  if (joint_positions.size() != num_joints_)
  {
    throw std::runtime_error("The size of joint_positions (" + std::to_string(joint_positions.size()) + ") is different from the number of joints (" +
                             std::to_string(num_joints_) + ").");
  }

  compute_frame_to_frame_transform(*kinematics_, joint_positions, desired_base_frame, end_effectors_frame, desired_base_to_end_effector_transform_);
}

void RTKinematicsSolver::throw_error_if_not_initialized_()
{
  if (!is_initialized_)
  {
    throw std::runtime_error("The RTKinematicsSolver class has not been initialized.");
  }
}

void compute_frame_to_frame_transform(kinematics_interface::KinematicsInterface& kinematics, const std::vector<double>& joint_positions,
                                      const std::string& start_frame, const std::string& end_frame, Eigen::Isometry3d& start_to_end_transform)
{
  Eigen::Isometry3d kinematics_base_to_start_frame_transform;
  Eigen::Isometry3d kinematics_base_to_end_frame_transform;

  // Compute kinematics_base_to_desired_base_transform and kinematics_base_to_end_effector_transform.
  // Note that if the desired base frame is the same as the kinematics base frame the transform is the identity
  if (!kinematics.calculate_link_transform(joint_positions, start_frame, kinematics_base_to_start_frame_transform) ||
      !kinematics.calculate_link_transform(joint_positions, end_frame, kinematics_base_to_end_frame_transform))
  {
    throw std::runtime_error("Failed to compute the frame to frame transform from " + start_frame + " to " + end_frame +
                             ". Check if the frame names are valid for the kinematics interface.");
  }

  // Compute the start to end transform
  start_to_end_transform = kinematics_base_to_start_frame_transform.inverse() * kinematics_base_to_end_frame_transform;
}

void transform_task_space_point_frames(kinematics_interface::KinematicsInterface& kinematics, const std::vector<double>& joint_positions,
                                       const std::string& desired_motion_frame, const std::string& desired_wrench_frame,
                                       acg_control_msgs::msg::TaskSpacePoint& task_space_point)
{
  if (!task_space_point.motion_frame.empty() && task_space_point.motion_frame != desired_motion_frame)
  {
    // Compute transform from desired motion frame to reference motion frame
    Eigen::Isometry3d desired_motion_to_reference_motion_transform;
    compute_frame_to_frame_transform(kinematics, joint_positions, desired_motion_frame, task_space_point.motion_frame,
                                     desired_motion_to_reference_motion_transform);

    // Update the pose of the task space reference
    Eigen::Isometry3d pose_transform;
    tf2::fromMsg(task_space_point.pose, pose_transform);
    pose_transform = desired_motion_to_reference_motion_transform * pose_transform;
    task_space_point.pose = tf2::toMsg(pose_transform);

    // Rotate twist and acceleration
    Eigen::Matrix<double, NUM_CARTESIAN_DOF, 1> twist, acceleration;
    tf2::fromMsg(task_space_point.twist, twist);
    tf2::fromMsg(task_space_point.acceleration, acceleration);

    // Note that the twist and acceleration must be only rotated, not translated.
    Eigen::Matrix<double, NUM_CARTESIAN_DOF, NUM_CARTESIAN_DOF> rotation_block = Eigen::Matrix<double, NUM_CARTESIAN_DOF, NUM_CARTESIAN_DOF>::Zero();
    rotation_block.block<3, 3>(0, 0) = rotation_block.block<3, 3>(3, 3) = desired_motion_to_reference_motion_transform.rotation();
    twist = rotation_block * twist;
    acceleration = rotation_block * acceleration;

    // Assign rotated twist and acceleration
    task_space_point.twist = tf2::toMsg(twist);
    tf2::toMsg(acceleration, task_space_point.acceleration);
  }

  // Update the wrench (force and torque) and wrench derivative of the task space reference
  if (!task_space_point.wrench_frame.empty() && task_space_point.wrench_frame != desired_wrench_frame)
  {
    // Update the wrench (force and torque) of the task space reference
    transform_wrench_frame(kinematics, joint_positions, desired_wrench_frame, task_space_point.wrench_frame, task_space_point.wrench);

    // Update the wrench derivative (force and torque) of the task space reference
    transform_wrench_frame(kinematics, joint_positions, desired_wrench_frame, task_space_point.wrench_frame, task_space_point.wrench_derivative);
  }

  // Update the motion frame and wrench frame of the task space reference with the desired ones
  task_space_point.wrench_frame = desired_wrench_frame;
  task_space_point.motion_frame = desired_motion_frame;
}

void transform_wrench_frame(kinematics_interface::KinematicsInterface& kinematics, const std::vector<double>& joint_positions,
                            const std::string& desired_wrench_frame, const std::string& wrench_frame, geometry_msgs::msg::Wrench& wrench)
{
  // Compute transform from desired to reference wrench frames
  Eigen::Isometry3d desired_wrench_to_reference_wrench_transform;
  compute_frame_to_frame_transform(kinematics, joint_positions, desired_wrench_frame, wrench_frame, desired_wrench_to_reference_wrench_transform);
  const Eigen::Vector3d& translation = desired_wrench_to_reference_wrench_transform.translation();
  const Eigen::Matrix3d& rotation = desired_wrench_to_reference_wrench_transform.rotation();

  // Update the wrench (force and torque)
  Eigen::Matrix<double, NUM_CARTESIAN_DOF, 1> wrench_eigen;
  tf2::fromMsg(wrench, wrench_eigen);

  // Compute the rotated force
  wrench_eigen.head(3) = rotation * wrench_eigen.head(3);

  // Compute the skew symmetric matrix using the vector that starts from desired to reference wrench frame
  Eigen::Matrix3d skew_symmetric_matrix;
  skew_symmetric_matrix << 0.0, -translation.z(), translation.y(), translation.z(), 0.0, -translation.x(), -translation.y(), translation.x(), 0.0;

  // Compute the rotated torque
  wrench_eigen.tail(3) = rotation * wrench_eigen.tail(3) + skew_symmetric_matrix * wrench_eigen.head(3);
  tf2::toMsg(wrench_eigen, wrench);
}

void compute_pose_error(const geometry_msgs::msg::Pose& desired_pose, const geometry_msgs::msg::Pose& current_pose,
                        Eigen::Matrix<double, 6, 1>& error)
{
  Eigen::Vector3d desired_position;
  Eigen::Vector3d current_position;
  Eigen::Vector3d position_error;
  Eigen::Quaterniond desired_quaternion;
  Eigen::Quaterniond current_quaternion;
  Eigen::Quaterniond error_quaternion;
  Eigen::Vector3d orientation_error;

  tf2::fromMsg(current_pose.position, current_position);
  tf2::fromMsg(current_pose.orientation, current_quaternion);
  tf2::fromMsg(desired_pose.position, desired_position);
  tf2::fromMsg(desired_pose.orientation, desired_quaternion);

  // Compute the position error
  position_error = desired_position - current_position;

  // Compute the orientation error
  error_quaternion = desired_quaternion * current_quaternion.inverse();

  Eigen::AngleAxisd error_angle_axis(error_quaternion);
  orientation_error = error_angle_axis.axis() * error_angle_axis.angle();

  // Set the error vector
  error.head(3) << position_error.x(), position_error.y(), position_error.z();
  error.tail(3) << orientation_error.x(), orientation_error.y(), orientation_error.z();
}

}  // namespace acg_kinematics
