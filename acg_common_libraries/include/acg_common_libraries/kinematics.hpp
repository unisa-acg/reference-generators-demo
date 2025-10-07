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

#include <acg_control_msgs/msg/task_space_point.hpp>

namespace acg_kinematics
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
                                  acg_control_msgs::msg::TaskSpacePoint& task_space_point);

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
                                       acg_control_msgs::msg::TaskSpacePoint& task_space_point);

/**
 * @brief Transform the wrench of the task space point to the desired frame.
 *
 * This function transforms the wrench of the task space point to be expressed in the desired wrench frame.
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
 * @brief Computes the error pose between the desired and current poses.
 *
 * @param[in] desired_pose The desired pose.
 * @param[in] current_pose The current pose.
 * @param[out] error The computed error pose, which is a 6D vector containing the position and orientation errors.
 */
void compute_pose_error(const geometry_msgs::msg::Pose& desired_pose, const geometry_msgs::msg::Pose& current_pose,
                        Eigen::Matrix<double, 6, 1>& error);

}  // namespace acg_kinematics
