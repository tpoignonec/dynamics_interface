// Copyright 2024 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
/// \authors: Thibault Poignonec

// Based on package "ros-controls/kinematics_interface_kdl", Copyright (c) 2022, PickNik, Inc.

#ifndef DYNAMICS_INTERFACE_KDL__DYNAMICS_INTERFACE_KDL_HPP_
#define DYNAMICS_INTERFACE_KDL__DYNAMICS_INTERFACE_KDL_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "dynamics_interface/dynamics_interface.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"
#include "kdl/chaindynparam.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/chainjnttojacdotsolver.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/treejnttojacsolver.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "tf2_eigen_kdl/tf2_eigen_kdl.hpp"

namespace dynamics_interface_kdl
{
class DynamicsInterfaceKDL : public dynamics_interface::DynamicsInterface
{
public:
  /**
   * \brief Initialize plugin. This method must be called before any other.
   * \param[in] robot_description robot URDF in string format
   * \param[in] parameters_interface
   * \param[in] param_namespace namespace for kinematics parameters - defaults to "kinematics"
   * \return true if successful
   */
  bool initialize(
    const std::string & robot_description,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
    const std::string & param_namespace) override;

  bool calculate_link_transform(
    const Eigen::VectorXd & joint_pos, const std::string & link_name,
    Eigen::Isometry3d & transform) override;

  bool calculate_jacobian(
    const Eigen::VectorXd & joint_pos, const std::string & link_name,
    Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian) override;

  bool calculate_jacobian_derivative(
    const Eigen::VectorXd & joint_pos, const Eigen::VectorXd & joint_vel,
    const std::string & link_name,
    Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian_derivative) override;

  bool calculate_jacobian_inverse(
    const Eigen::VectorXd & joint_pos, const std::string & link_name,
    Eigen::Matrix<double, Eigen::Dynamic, 6> & jacobian_inverse) override;

  bool calculate_inertia(
    const Eigen::VectorXd & joint_pos,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & inertia) override;

  bool calculate_coriolis(
    const Eigen::VectorXd & joint_pos, const Eigen::VectorXd & joint_vel,
    Eigen::VectorXd & coriolis) override;

  bool calculate_gravity(const Eigen::VectorXd & joint_pos, Eigen::VectorXd & gravity) override;

  bool convert_cartesian_deltas_to_joint_deltas(
    const Eigen::VectorXd & joint_pos, const Eigen::Matrix<double, 6, 1> & delta_x,
    const std::string & link_name, Eigen::VectorXd & delta_theta) override;

  bool convert_joint_deltas_to_cartesian_deltas(
    const Eigen::VectorXd & joint_pos, const Eigen::VectorXd & delta_theta,
    const std::string & link_name, Eigen::Matrix<double, 6, 1> & delta_x) override;

private:
  // verification methods
  bool verify_initialized();
  bool verify_link_name(const std::string & link_name);
  bool verify_joint_vector(const Eigen::VectorXd & joint_vector);
  bool verify_jacobian(const Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian);
  bool verify_jacobian_inverse(const Eigen::Matrix<double, Eigen::Dynamic, 6> & jacobian);
  bool verify_inertia(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & inertia);
  bool verify_coriolis(const Eigen::VectorXd & coriolis);
  bool verify_gravity(const Eigen::VectorXd & gravity);

  bool initialized = false;
  std::string root_name_;
  KDL::Vector gravity_in_base_frame_;
  size_t num_joints_;
  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
  KDL::JntArray q_, q_dot_;
  KDL::JntArrayVel q_array_vel_;
  KDL::Frame frame_;
  std::shared_ptr<KDL::Jacobian> jacobian_, jacobian_derivative_;
  std::shared_ptr<Eigen::Matrix<double, Eigen::Dynamic, 6>> jacobian_inverse_;
  std::shared_ptr<KDL::JntSpaceInertiaMatrix> inertia_;
  KDL::JntArray coriolis_, gravity_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::shared_ptr<KDL::ChainJntToJacDotSolver> jac_dot_solver_;
  std::shared_ptr<KDL::ChainDynParam> dyn_solver_;
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;
  std::unordered_map<std::string, int> link_name_map_;
  double alpha;  // damping term for Jacobian inverse
  Eigen::MatrixXd I;
};

}  // namespace dynamics_interface_kdl

#endif  // DYNAMICS_INTERFACE_KDL__DYNAMICS_INTERFACE_KDL_HPP_
