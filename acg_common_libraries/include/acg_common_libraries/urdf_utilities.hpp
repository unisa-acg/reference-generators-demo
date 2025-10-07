/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   urdf_utilities.hpp
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Mar 4, 2025
 *
 * This class contains utility functions for managing the properties
 * of the URDF.
 *
 * -------------------------------------------------------------------
 */

#pragma once

// Eigen
#include <eigen3/Eigen/Core>

// URDF
#include <urdf/model.h>

namespace urdf_utilities
{
/**
 * @brief Compute the mass of a chain of links starting from a specified link.
 * The mass is computed as the sum of the mass of the link itself and the mass of its children links.
 *
 * @param[in] link the starting link of the chain
 *
 * @return the chain's mass
 */
double compute_chain_mass(const urdf::LinkConstSharedPtr& link);

/**
 * @brief Compute the mass and center of mass (COM) of a chain of links starting from a specified link.
 * The mass is computed as the sum of the mass of the link itself and the mass of its children links.
 * The COM is computed as the weighted average of the COM of the link itself and the COM of its children links.
 *
 * @param[in] link the starting link of the chain
 *
 * @return the chain's mass and COM
 */
std::pair<double, Eigen::Vector3d> compute_chain_mass_and_com(const urdf::LinkConstSharedPtr& link);

/**
 * @brief Perform a depth-first traversal and apply the map-reduce pattern over a URDF link tree.
 *
 * This function recursively visits each link in the subtree rooted at @p{link},
 * applies @p map_func to each link to obtain a value of type T, and combines
 * the results using @p{reduce_func}, starting from @p{initial_value}.
 *
 * @tparam T The type of the accumulated result.
 * @param link The root link to start traversal from.
 * @param map_func Function to map each link to a value of type T.
 * @param initial_value The initial value for the reduction.
 * @param reduce_func Function to combine two values of type T.
 * @return The accumulated result after traversing and reducing all links.
 */
template <typename T>
T dfs_map_reduce_links(const urdf::LinkConstSharedPtr& link, const std::function<T(const urdf::LinkConstSharedPtr&)>& map_func,
                       const T& initial_value, const std::function<T(const T&, const T&)>& reduce_func)
{
  // If the link is null, return the initial value
  if (!link)
    return initial_value;

  // Initialize the reduced value with the value extracted from the current link
  T reduced_value = map_func(link);

  // If there are no child links, return the reduced value
  if (link->child_links.empty())
  {
    return reduced_value;
  }

  // Traverse all child links and reduce the results
  for (auto& child_link : link->child_links)
  {
    reduced_value = reduce_func(reduced_value, dfs_map_reduce_links<T>(child_link, map_func, initial_value, reduce_func));
  }

  return reduced_value;
}

/**
 * @brief Exception thrown when joint limits are not defined in the URDF.
 */
class JointLimitsNotDefinedException : public std::runtime_error
{
public:
  JointLimitsNotDefinedException(const std::string& msg) : std::runtime_error(msg) {}
};

/**
 * @brief Read the joint position limits from the URDF.
 * The limits are stored in the provided vectors, where each vector corresponds to a joint.
 * If a joint does not have limits defined, the upper limit is set to the maximum double value and the lower limit to the minimum double value and
 * an exception is thrown.
 *
 * @param[in] robot_description the URDF string of the robot
 * @param[in] joint_names the names of the joints to read limits for
 * @param[out] joint_positions_upper_limits_ the vector to store the upper limits of the joint positions. The vector is resized to match the number of
 * joints.
 * @param[out] joint_positions_lower_limits_ the vector to store the lower limits of the joint positions. The vector is resized to match the number of
 * joints.
 *
 * @throws std::runtime_error if a joint is not found in the URDF.compute_chain_mass
 * @throws JointLimitsNotDefinedException if any joint does not have limits defined. In the latter case, the error message
 * will contain the names of the joints without limits.
 */
void read_joint_position_limits_from_urdf(const std::string& robot_description, const std::vector<std::string>& joint_names,
                                          std::vector<double>& joint_positions_upper_limits_, std::vector<double>& joint_positions_lower_limits_);

/**
 * @brief Read the joint velocity limits from the URDF.
 * The limits are stored in the provided vector, where each element corresponds to a joint.
 * If a joint does not have limits defined, the limit is set to the maximum double value and an exception is thrown.
 *
 * @param[in] robot_description the URDF string of the robot
 * @param[in] joint_names the names of the joints to read limits for
 * @param[out] joint_velocity_limits_ the vector to store the velocity limits of the joints. The vector is resized to match the number of joints.
 *
 * @throws std::runtime_error if a joint is not found in the URDF.
 * @throws JointLimitsNotDefinedException if any joint does not have limits defined. In the latter case, the error message
 * will contain the names of the joints without limits.
 */
void read_joint_velocity_limits_from_urdf(const std::string& robot_description, const std::vector<std::string>& joint_names,
                                          std::vector<double>& joint_velocity_limits_);

/**
 * @brief Read the joint effort limits from the URDF.
 * The limits are stored in the provided vector, where each element corresponds to a joint.
 * If a joint does not have limits defined, the limit is set to the maximum double value and an exception is thrown.
 *
 * @param[in] robot_description the URDF string of the robot
 * @param[in] joint_names the names of the joints to read limits for
 * @param[out] joint_effort_limits_ the vector to store the effort limits of the joints. The vector is resized to match the number of joints.
 *
 * @throws std::runtime_error if a joint is not found in the URDF.
 * @throws JointLimitsNotDefinedException if any joint does not have limits defined. In the latter case, the error message
 * will contain the names of the joints without limits.
 */
void read_joint_effort_limits_from_urdf(const std::string& robot_description, const std::vector<std::string>& joint_names,
                                        std::vector<double>& joint_effort_limits_);

}  // namespace urdf_utilities
