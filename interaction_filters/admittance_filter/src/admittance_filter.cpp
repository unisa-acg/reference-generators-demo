/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   admittance_filter.cpp
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Feb 4, 2025
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <pluginlib/class_list_macros.hpp>
#include <acg_common_libraries/message_utilities.hpp>
#include "admittance_filter/admittance_filter.hpp"

namespace interaction_filters
{
AdmittanceFilter::~AdmittanceFilter()
{
  reset();
}

bool AdmittanceFilter::update(const acg_control_msgs::msg::TaskSpacePoint& task_space_reference, const rclcpp::Duration& delta_t,
                              acg_control_msgs::msg::TaskSpacePoint& task_space_command)
{
  if (!is_initialized())
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "%s not initialized", filter_name_.c_str());
    return false;
  }

  // Update the filter parameters
  apply_parameters_update();

  // Get the reference values
  Vector6d x_d, x_dot_d, x_dot_dot_d, delta_h;
  tf2::fromMsg(task_space_reference.pose, x_d);
  tf2::fromMsg(task_space_reference.twist, x_dot_d);
  tf2::fromMsg(task_space_reference.acceleration, x_dot_dot_d);
  tf2::fromMsg(task_space_reference.wrench, delta_h);

  // Compute the admittance law
  switch (order_)
  {
    case 0:
    {
      x_dot_dot_c_ = x_dot_dot_d;
      x_dot_c_ = x_dot_d;
      x_c_ = -1 * K_P_.inverse() * (-delta_h - K_P_ * x_d);
      break;
    }
    case 1:
    {
      x_dot_dot_c_ = x_dot_dot_d;
      x_dot_c_ = -1 * K_D_.inverse() * (-delta_h - K_D_ * x_dot_d - K_P_ * (x_d - x_c_));
      x_c_ = x_dot_c_ * delta_t.seconds() + x_c_;
      break;
    }
    case 2:
    {
      x_dot_dot_c_ = x_dot_dot_d - M_d_inv_ * (-delta_h - K_D_ * (x_dot_d - x_dot_c_) - K_P_ * (x_d - x_c_));
      x_dot_c_ = x_dot_dot_c_ * delta_t.seconds() + x_dot_c_;
      x_c_ = x_dot_c_ * delta_t.seconds() + x_c_;
      break;
    }
  }

  // Zero out the non-compliant axes
  x_dot_dot_c_ = x_dot_dot_c_.cwiseProduct(S_) + x_dot_dot_d.cwiseProduct(Vector6d::Ones() - S_);
  x_dot_c_ = x_dot_c_.cwiseProduct(S_) + x_dot_d.cwiseProduct(Vector6d::Ones() - S_);
  x_c_ = x_c_.cwiseProduct(S_) + x_d.cwiseProduct(Vector6d::Ones() - S_);

  tf2::toMsg(x_c_, task_space_command.pose);
  tf2::toMsg(x_dot_c_, task_space_command.twist);
  tf2::toMsg(x_dot_dot_c_, task_space_command.acceleration);

  return true;
}

bool AdmittanceFilter::reset()
{
  M_d_inv_.setZero();
  K_D_.setZero();
  K_P_.setZero();
  x_c_.setZero();
  x_dot_c_.setZero();
  x_dot_dot_c_.setZero();
  S_.setZero();
  return true;
}

void AdmittanceFilter::apply_parameters_update()
{
  // Get filter parameters
  if (parameter_handler_->is_old(parameters_))
  {
    parameters_ = parameter_handler_->get_params();
  }

  order_ = parameters_.order;

  Vector6d M_d(parameters_.mass.data());
  M_d_inv_ = M_d.cwiseInverse().asDiagonal();
  K_P_ = Vector6d(parameters_.stiffness.data()).asDiagonal();
  K_D_ = 2 * Vector6d(parameters_.damping_ratio.data()).cwiseProduct(M_d.cwiseProduct(K_P_.diagonal()).cwiseSqrt()).asDiagonal();

  std::transform(parameters_.compliant_axis.begin(), parameters_.compliant_axis.end(), S_.data(), [](bool val) { return static_cast<double>(val); });
}

bool AdmittanceFilter::initialize_()
{
  // Initialize the filter variables
  reset();

  // Initialize the parameter handler
  parameter_handler_ = std::make_shared<admittance_filter::ParamListener>(params_interface_, logging_interface_->get_logger(), filter_name_);

  // Update filter parameters
  apply_parameters_update();
  return true;
}
}  // namespace interaction_filters

PLUGINLIB_EXPORT_CLASS(interaction_filters::AdmittanceFilter, interaction_filters::InteractionFilterBase)
