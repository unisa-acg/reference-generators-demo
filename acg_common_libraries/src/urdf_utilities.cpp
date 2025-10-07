#include <functional>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>

#include <urdf_model/pose.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "acg_common_libraries/urdf_utilities.hpp"

double urdf_utilities::compute_chain_mass(const urdf::LinkConstSharedPtr& link)
{
  const std::function<double(const urdf::LinkConstSharedPtr&)> get_mass_from_link = [](const urdf::LinkConstSharedPtr& link)
  { return link->inertial ? link->inertial->mass : 0.0; };

  return urdf_utilities::dfs_map_reduce_links<double>(link, get_mass_from_link, 0.0, std::plus<double>());
}

std::pair<double, Eigen::Vector3d> urdf_utilities::compute_chain_mass_and_com(const urdf::LinkConstSharedPtr& link)
{
  using ReduceType = std::tuple<double, Eigen::Vector3d, Eigen::Affine3d>;

  const std::function<ReduceType(const urdf::LinkConstSharedPtr&)> map_func = [](const urdf::LinkConstSharedPtr& link)
  {
    if (!link or !link->inertial)
    {
      return std::make_tuple(0.0, Eigen::Vector3d(Eigen::Vector3d::Zero()), Eigen::Affine3d::Identity());
    }
    double mass = link->inertial->mass;
    double com_x = link->inertial->origin.position.x;
    double com_y = link->inertial->origin.position.y;
    double com_z = link->inertial->origin.position.z;
    Eigen::Vector3d com{ com_x, com_y, com_z };

    if (!link->parent_joint)
    {
      return std::make_tuple(0.0, com, Eigen::Affine3d::Identity());
    }

    const urdf::Vector3& translation = link->parent_joint->parent_to_joint_origin_transform.position;
    const urdf::Rotation& rotation = link->parent_joint->parent_to_joint_origin_transform.rotation;
    Eigen::Quaterniond rotation_eigen(rotation.w, rotation.x, rotation.y, rotation.z);
    Eigen::Translation3d translation_eigen(translation.x, translation.y, translation.z);
    Eigen::Affine3d parent_to_current_transform_eigen = translation_eigen * rotation_eigen;

    return std::make_tuple(mass, com, parent_to_current_transform_eigen);
  };

  const std::function<ReduceType(ReduceType, ReduceType)> reduce_func = [](ReduceType parent, ReduceType child)
  {
    double mass_parent = std::get<0>(parent);
    double mass_child = std::get<0>(child);
    Eigen::Vector3d com_parent = std::get<1>(parent);
    Eigen::Vector3d com_child = std::get<1>(child);
    Eigen::Affine3d transform_parent = std::get<2>(parent);
    Eigen::Affine3d transform_child = std::get<2>(child);

    // Transform the child's COM into the parent's frame
    com_child = transform_child * com_child;

    // Reduce the masses and the COMs
    double total_mass = mass_parent + mass_child;
    Eigen::Vector3d weighted_com = (com_parent * mass_parent + com_child * mass_child) / total_mass;

    return std::make_tuple(total_mass, weighted_com, transform_parent);
  };

  ReduceType initial_value = std::make_tuple(0.0, Eigen::Vector3d::Zero(), Eigen::Affine3d::Identity());
  ReduceType result = urdf_utilities::dfs_map_reduce_links<ReduceType>(link, map_func, initial_value, reduce_func);

  return std::make_pair(std::get<0>(result), std::get<1>(result));
}

namespace
{
// Helper function to read joint limits. This function is not meant to be visible outside this translation unit so it is defined in the anonymous
// namespace.
void read_joint_limits_from_urdf(const std::string& robot_description, const std::vector<std::string>& joint_names,
                                 std::function<void(const urdf::JointLimits&, std::size_t)> set_limit, std::function<void(std::size_t)> set_default)
{
  urdf::Model robot_model;
  if (!robot_model.initString(robot_description))
    throw std::runtime_error("Failed to parse the URDF.");

  std::size_t num_joints{ joint_names.size() };
  std::vector<std::string> joints_without_limits;

  for (std::size_t i = 0; i < num_joints; ++i)
  {
    const std::string& joint_name{ joint_names[i] };
    urdf::JointConstSharedPtr joint{ robot_model.getJoint(joint_name) };

    if (!joint)
      throw std::runtime_error("Joint not found in URDF: " + joint_name);

    if (joint->limits)
      set_limit(*joint->limits, i);
    else
    {
      joints_without_limits.push_back(joint_name);
      set_default(i);
    }
  }

  if (!joints_without_limits.empty())
  {
    std::string error_message = "The following joints do not have limits defined: ";
    for (const std::string& joint_name : joints_without_limits)
    {
      error_message += joint_name + ", ";
    }
    error_message.pop_back();  // Remove the last comma
    error_message.pop_back();  // Remove the last space
    throw urdf_utilities::JointLimitsNotDefinedException(error_message);
  }
}
}  // namespace

void urdf_utilities::read_joint_position_limits_from_urdf(const std::string& robot_description, const std::vector<std::string>& joint_names,
                                                          std::vector<double>& upper_limits, std::vector<double>& lower_limits)
{
  upper_limits.resize(joint_names.size());
  lower_limits.resize(joint_names.size());
  read_joint_limits_from_urdf(
      robot_description, joint_names,
      [&](const urdf::JointLimits& limits, std::size_t i)
      {
        upper_limits[i] = limits.upper;
        lower_limits[i] = limits.lower;
      },
      [&](std::size_t i)
      {
        upper_limits[i] = std::numeric_limits<double>::max();
        lower_limits[i] = std::numeric_limits<double>::lowest();
      });
}

void urdf_utilities::read_joint_velocity_limits_from_urdf(const std::string& robot_description, const std::vector<std::string>& joint_names,
                                                          std::vector<double>& velocity_limits)
{
  velocity_limits.resize(joint_names.size());
  read_joint_limits_from_urdf(
      robot_description, joint_names, [&](const urdf::JointLimits& limits, std::size_t i) { velocity_limits[i] = limits.velocity; },
      [&](std::size_t i) { velocity_limits[i] = std::numeric_limits<double>::max(); });
}

void urdf_utilities::read_joint_effort_limits_from_urdf(const std::string& robot_description, const std::vector<std::string>& joint_names,
                                                        std::vector<double>& effort_limits)
{
  effort_limits.resize(joint_names.size());
  read_joint_limits_from_urdf(
      robot_description, joint_names, [&](const urdf::JointLimits& limits, std::size_t i) { effort_limits[i] = limits.effort; },
      [&](std::size_t i) { effort_limits[i] = std::numeric_limits<double>::max(); });
}
