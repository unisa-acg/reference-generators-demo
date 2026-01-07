/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   admittance_filter.hpp
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Feb 4, 2025
 *
 * This class implements all the methods to use an admittance filter.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <memory>
#include <Eigen/Dense>
#include <interaction_filter_base/interaction_filter_base.hpp>
#include <admittance_filter/admittance_filter_parameters.hpp>

namespace interaction_filters
{
static const unsigned short int NUM_CARTESIAN_DOF = 6;
typedef Eigen::DiagonalMatrix<double, NUM_CARTESIAN_DOF, NUM_CARTESIAN_DOF> DiagonalMatrix6d;
typedef Eigen::Matrix<double, NUM_CARTESIAN_DOF, 1> Vector6d;

class AdmittanceFilter : public interaction_filters::InteractionFilterBase
{
public:
  /**
   * @brief Refer to the superclass documentation.
   */
  ~AdmittanceFilter() override;

  /**
   * @brief Refer to the superclass documentation.
   */
  bool update(const xxx_control_msgs::msg::TaskSpacePoint& task_space_reference, const rclcpp::Duration& delta_t,
              xxx_control_msgs::msg::TaskSpacePoint& task_space_command) override;

  /**
   * @brief Refer to the superclass documentation.
   */
  bool reset() override;

  /**
   * @brief Refer to the superclass documentation.
   */
  void apply_parameters_update() override;

protected:
  /**
   * @brief Refer to the superclass documentation.
   */
  bool initialize_() override;

private:
  /**
   * @brief Pointer to the parameter handler.
   */
  std::shared_ptr<admittance_filter::ParamListener> parameter_handler_;

  /**
   * @brief Filter parameters.
   */
  admittance_filter::Params parameters_;

  /**
   * @brief Inverse mass diagonal matrix.
   */
  DiagonalMatrix6d M_d_inv_;

  /**
   * @brief Damping diagonal matrix.
   */
  DiagonalMatrix6d K_D_;

  /**
   * @brief Stiffness diagonal matrix.
   */
  DiagonalMatrix6d K_P_;

  /**
   * @brief Task space pose command.
   */
  Vector6d x_c_;

  /**
   * @brief Task space twist command.
   */
  Vector6d x_dot_c_;

  /**
   * @brief Task space acceleration command.
   */
  Vector6d x_dot_dot_c_;

  /**
   * @brief Order of the filter in the set {0, 1, 2}.
   */
  unsigned short int order_;

  /**
   * @brief Vector of compliant axes, where 1.0 means compliant and 0.0 means stiff.
   */
  Vector6d S_;
};

}  // namespace interaction_filters
