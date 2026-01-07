/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   kinematics.hpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Feb 10, 2025
 *
 * This library contains functions for kinematics.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // for tf2::fromMsg

#include <kinematics_interface/kinematics_interface.hpp>

#include <xxx_control_msgs/msg/task_space_point.hpp>

namespace xxx_kinematics
{

// This constant defines the number of degrees of freedom in the Cartesian space.
constexpr short unsigned int NUM_CARTESIAN_DOF = 6;

class RTKinematicsSolver
{
public:
  RTKinematicsSolver() = default;
  ~RTKinematicsSolver() = default;

  /**
   * @brief Initialize the kinematics object.
   *
   * @param[in] num_joints The number of joints of the robot.
   * @param[in] kinematics A shared pointer to the kinematics interface of the robot.
   */
  void initialize(const std::size_t num_joints, const std::shared_ptr<kinematics_interface::KinematicsInterface>& kinematics);

  /**
   * @brief Compute the zero-order forward kinematics of the robot.
   *
   * This function computes the forward kinematics of the robot given the joint positions, the end effector frame, and the
   * desired base frame, storing the result in the \p pose variable. It can be used in real-time context. The remaining fields of
   * the task space point are not computed so they will not be set by this function.
   *
   * @param[in] joint_positions The joint positions of the robot.
   * @param[in] joint_velocities The joint velocities of the robot. If the joint velocities are not available, pass an empty vector.
   * @param[in] end_effectors_frame The end effector frame.
   * @param[in] desired_base_frame The desired base frame.
   * @param[in,out] pose The pose to store the result.
   *
   */
  void compute_forward_kinematics(const std::vector<double>& joint_positions, const std::string& end_effectors_frame,
                                  const std::string& desired_base_frame, geometry_msgs::msg::Pose& pose);

  /**
   * @brief Compute the zero-order and first-order forward kinematics of the robot.
   *
   * This function computes the forward kinematics of the robot given the joint positions and joint velocities, the end effector frame, and the
   * desired base frame, storing the result in the \p task_space_point variable. It can be used in real-time context. Note that this function sets the
   * pose of the task space point to the desired base frame and, if the joint velocities are available, computes the twist. The remaining fields of
   * the task space point are not computed so they will not be set by this function.
   *
   * @param[in] joint_positions The joint positions of the robot.
   * @param[in] joint_velocities The joint velocities of the robot. If the joint velocities are not available, pass an empty vector.
   * @param[in] end_effectors_frame The end effector frame.
   * @param[in] desired_base_frame The desired base frame.
   * @param[in,out] task_space_point The task space point to store the result.
   *
   */
  void compute_forward_kinematics(const std::vector<double>& joint_positions, const std::vector<double>& joint_velocities,
                                  const std::string& end_effectors_frame, const std::string& desired_base_frame,
                                  xxx_control_msgs::msg::TaskSpacePoint& task_space_point);

protected:
  bool is_initialized_{ false };
  std::size_t num_joints_{ 0 };
  Eigen::Matrix<double, NUM_CARTESIAN_DOF, Eigen::Dynamic> jacobian_;
  std::shared_ptr<kinematics_interface::KinematicsInterface> kinematics_;
  Eigen::Isometry3d desired_base_to_end_effector_transform_;

  /**
   * @brief Compute the transform from the base frame to the desired base frame and the transform from the base frame to the end effector frame.
   *
   * This function computes the transform from the base frame to the desired base frame and the transform from the base frame to the end effector
   * frame. It is used internally by the compute_forward_kinematics function.
   *
   * @throws \c std::runtime_error if the size of joint_positions is different from the number of joints.
   *
   * @param[in] joint_positions The joint positions of the robot.
   * @param[in] end_effectors_frame The end effector frame.
   * @param[in] desired_base_frame The desired base frame.
   */
  void compute_desired_base_to_end_effector_transform_(const std::vector<double>& joint_positions, const std::string& end_effectors_frame,
                                                       const std::string& desired_base_frame);

  /**
   * @brief Verify if the kinematics object is initialized.
   *
   * @throws std::runtime_error if the kinematics object is not initialized.
   */
  void throw_error_if_not_initialized_();
};

/**
 * @brief Compute the transform between two specified frames of the kinematics interface.
 *
 * This function computes the transform from a start frame to an end frame, given the robot's joint positions.
 * It is used internally by the compute_forward_kinematics function to determine the relationship between frames.
 *
 * @throws \c std::runtime_error if the size of \p joint_positions is different from the number of joints.
 *
 * @param[in] kinematics The kinematics interface of the robot.
 * @param[in] joint_positions The joint positions of the robot.
 * @param[in] start_frame The name of the starting frame.
 * @param[in] end_frame The name of the target frame.
 * @param[out] start_to_end_transform The computed transform from the start frame to the end frame.
 */
void compute_frame_to_frame_transform(kinematics_interface::KinematicsInterface& kinematics, const std::vector<double>& joint_positions,
                                      const std::string& start_frame, const std::string& end_frame, Eigen::Isometry3d& start_to_end_transform);

/**
 * @brief Transform the task space point to be expressed in the desired motion and wrench frames.
 *
 * This function transforms the task space point such that it is expressed in the specified motion frame and wrench frame.
 * The pose, twist, acceleration, wrench and wrench derivative of the task space point are updated accordingly.
 *
 * @param[in] kinematics The robot's kinematics interface.
 * @param[in] joint_positions The robot's joint positions.
 * @param[in] desired_motion_frame The target motion frame to express the task space point in.
 * @param[in] desired_wrench_frame The target wrench frame.
 * @param[in,out] task_space_point The task space point whose frame is to be transformed. It is updated with the new frame information.
 */
void transform_task_space_point_frames(kinematics_interface::KinematicsInterface& kinematics, const std::vector<double>& joint_positions,
                                       const std::string& desired_motion_frame, const std::string& desired_wrench_frame,
                                       xxx_control_msgs::msg::TaskSpacePoint& task_space_point);

/**
 * @brief Transform the wrench to the desired frame.
 *
 * This function transforms the wrench to be expressed in the desired wrench frame.
 * The force and torque components of the wrench are updated accordingly.
 *
 * @param[in] kinematics The robot's kinematics interface.
 * @param[in] joint_positions The robot's joint positions.
 * @param[in] desired_wrench_frame The target wrench frame for the transformation.
 * @param[in] wrench_frame The current wrench frame.
 * @param[in,out] wrench The wrench to be transformed. It is updated with the new frame.
 */
void transform_wrench_frame(kinematics_interface::KinematicsInterface& kinematics, const std::vector<double>& joint_positions,
                            const std::string& desired_wrench_frame, const std::string& wrench_frame, geometry_msgs::msg::Wrench& wrench);

/**
 * @brief Transform the wrench to the desired frame.
 *
 * This function transforms the wrench to be expressed in the desired wrench frame.
 * The force and torque components of the wrench are updated accordingly.
 * The current wrench frame must be specified in the \c frame_id field of the message's header.
 *
 * @param[in] kinematics The robot's kinematics interface.
 * @param[in] joint_positions The robot's joint positions.
 * @param[in] desired_wrench_frame The target wrench frame for the transformation.
 * @param[in,out] wrench The wrench to be transformed. It is updated with the new frame.
 */
void transform_wrench_frame(kinematics_interface::KinematicsInterface& kinematics, const std::vector<double>& joint_positions,
                            const std::string& desired_wrench_frame, geometry_msgs::msg::WrenchStamped& wrench);

/**
 * @brief Transforms the wrench to the desired frame.
 *
 * This function transforms the wrench to be expressed in the desired frame,
 * represented by the desired transform.
 * The force and torque components of the wrench are updated accordingly.
 *
 * @param[in] kinematics The robot's kinematics interface.
 * @param[in] joint_positions The robot's joint positions.
 * @param[in] desired_transform The desired transform expressed in the motion frame.
 * @param[in] motion_frame The frame with respect to which the desired transform is defined.
 * @param[in] wrench_frame The current wrench frame.
 * @param[in,out] wrench The wrench to be transformed. It is updated with the new frame.
 */
void transform_wrench_frame(kinematics_interface::KinematicsInterface& kinematics, const std::vector<double>& joint_positions,
                            const geometry_msgs::msg::Transform& desired_transform, const std::string& motion_frame, const std::string& wrench_frame,
                            geometry_msgs::msg::Wrench& wrench);

/**
 * @brief Transforms the wrench to the desired frame.
 *
 * This function transforms the wrench to be expressed in the desired frame,
 * represented by the desired transform.
 * The force and torque components of the wrench are updated accordingly.
 * The current wrench frame and the transform frame must be specified in the \c frame_id field of the relative message's header.
 *
 * @param[in] kinematics The robot's kinematics interface.
 * @param[in] joint_positions The robot's joint positions.
 * @param[in] desired_transform The desired transform expressed in the motion frame.
 * @param[in,out] wrench The wrench to be transformed. It is updated with the new frame.
 */
void transform_wrench_frame(kinematics_interface::KinematicsInterface& kinematics, const std::vector<double>& joint_positions,
                            const geometry_msgs::msg::TransformStamped& desired_transform, geometry_msgs::msg::WrenchStamped& wrench);

/**
 * @brief Computes the error pose between the desired and current poses using KDL.
 *
 * @param[in] desired_pose The desired pose.
 * @param[in] current_pose The current pose.
 * @param[out] error The computed error pose, which is a 6D vector containing the position and orientation errors.
 */
void compute_pose_error(const geometry_msgs::msg::Pose& desired_pose, const geometry_msgs::msg::Pose& current_pose,
                        Eigen::Matrix<double, 6, 1>& error);

/**
 * @brief Computes the error twist between the desired and current twists.
 *
 * @param[in] desired_twist The desired twist.
 * @param[in] current_twist The current twist.
 * @param[out] error The computed error twist, which is a 6D vector containing the linear and angular velocity errors.
 */
void compute_twist_error(const geometry_msgs::msg::Twist& desired_twist, const geometry_msgs::msg::Twist& current_twist,
                         Eigen::Matrix<double, 6, 1>& error);

/**
 * @brief Computes the error wrench between the desired and current wrenches.
 *
 * @param[in] desired_wrench The desired wrench.
 * @param[in] current_wrench The current wrench.
 * @param[out] error The computed error wrench, which is a 6D vector containing the force and momentum errors.
 */
void compute_wrench_error(const geometry_msgs::msg::Wrench& desired_wrench, const geometry_msgs::msg::Wrench& current_wrench,
                          Eigen::Matrix<double, 6, 1>& error);

/**
 * @brief Computes the Euclidean distance between two points using the L2 norm.
 *
 * @param[in] point1 The first point.
 * @param[in] point2 The second point.
 * @return The Euclidean distance between the two points in meters.
 */
double compute_euclidean_distance(const geometry_msgs::msg::Point& point1, const geometry_msgs::msg::Point& point2);

/**
 * @brief Computes the angular distance between two quaternions.
 *
 * This function computes the angular distance between two quaternions using the Eigen library.
 * The angular distance is defined as the angle of the rotation, in the axis–angle representation, that transforms one quaternion into the other.
 *
 * @param[in] quat1 The first quaternion.
 * @param[in] quat2 The second quaternion.
 * @return The angular distance between the two quaternions in radians.
 */
double compute_angular_distance(const geometry_msgs::msg::Quaternion& quat1, const geometry_msgs::msg::Quaternion& quat2);

/**
 * @brief Checks if two poses are close to each other within specified tolerances.
 *
 * This function checks if the translational and rotational distances between two poses are within the given tolerances.
 * It uses the compute_euclidean_distance and compute_angular_distance functions to calculate the distances.
 *
 * @param[in] pose1 The first pose.
 * @param[in] pose2 The second pose.
 * @param[in] translational_tolerance The maximum allowable translational distance in meters.
 * @param[in] rotational_tolerance The maximum allowable rotational distance in radians.
 * @return true if the poses are close within the specified tolerances, false otherwise.
 */
bool is_pose_close(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2, const double translational_tolerance,
                   const double rotational_tolerance);

}  // namespace xxx_kinematics
