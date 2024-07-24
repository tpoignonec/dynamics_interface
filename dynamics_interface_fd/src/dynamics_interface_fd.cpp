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

#include "dynamics_interface_fd/dynamics_interface_fd.hpp"

namespace dynamics_interface_fd
{
rclcpp::Logger LOGGER = rclcpp::get_logger("dynamics_interface_fd");

bool DynamicsInterfaceFd::initialize(
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
  const std::string & end_effector_name)
{
  // track initialization plugin
  initialized = true;

  // get robot description
  auto robot_param = rclcpp::Parameter();
  if (!parameters_interface->get_parameter("robot_description", robot_param))
  {
    RCLCPP_ERROR(LOGGER, "parameter robot_description not set");
    return false;
  }
  auto robot_description = robot_param.as_string();

  // get alpha damping term
  auto alpha_param = rclcpp::Parameter("alpha", 0.000005);
  if (parameters_interface->has_parameter("alpha"))
  {
    parameters_interface->get_parameter("alpha", alpha_param);
  }
  alpha = alpha_param.as_double();

  // create kinematic chain
  KDL::Tree robot_tree;
  kdl_parser::treeFromString(robot_description, robot_tree);

  // get root name
  auto base_param = rclcpp::Parameter();
  if (parameters_interface->has_parameter("base"))
  {
    parameters_interface->get_parameter("base", base_param);
    root_name_ = base_param.as_string();
  }
  else
  {
    root_name_ = robot_tree.getRootSegment()->first;
  }

  if (!robot_tree.getChain(root_name_, end_effector_name, chain_))
  {
    RCLCPP_ERROR(
      LOGGER, "failed to find chain from robot root %s to end effector %s", root_name_.c_str(),
      end_effector_name.c_str());
    return false;
  }

  // create map from link names to their index
  for (size_t i = 0; i < chain_.getNrOfSegments(); ++i)
  {
    link_name_map_[chain_.getSegment(i).getName()] = i + 1;
  }

  // allocate dynamics memory
  num_joints_ = chain_.getNrOfJoints();
  q_ = KDL::JntArray(num_joints_);
  q_dot_ = KDL::JntArray(num_joints_);
  q_array_vel_ = KDL::JntArrayVel(num_joints_);  // container for for q AND q_dot_
  I = Eigen::MatrixXd(num_joints_, num_joints_);
  I.setIdentity();

  jacobian_ = std::make_shared<KDL::Jacobian>(num_joints_);
  jacobian_derivative_ = std::make_shared<KDL::Jacobian>(num_joints_);

  // create KDL solvers
  fk_pos_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
  jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);
  jac_dot_solver_ = std::make_shared<KDL::ChainJntToJacDotSolver>(chain_);

  return true;
}

bool DynamicsInterfaceFd::fill_joints_if_missing(
  const Eigen::VectorXd & joint_values,
  Eigen::VectorXd & joint_values_filled)
{
  if (joint_values.size() == 0 || static_cast<size_t>(joint_values.size()) > num_joints_) {
    // TODO(tpoignonec) add warning
    return false;
  }

  if (joint_values.size() != 3 && joint_values.size() != 6) {
    // should be either 3 or 6!
    // TODO(tpoignonec) add warning
    return false;
  }

  if (static_cast<size_t>(joint_values.size()) == num_joints_) {
    joint_values_filled = joint_values;
    return true;
  }

  joint_values_filled = Eigen::VectorXd::Zero(num_joints_);
  joint_values_filled.head(joint_values.size()) = joint_values;
  return true;
}

// FK, Jacobian, and Jacobian time derivative
// --------------------------------------------------------------------

bool DynamicsInterfaceFd::calculate_link_transform(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos, const std::string & link_name,
  Eigen::Isometry3d & transform)
{
  // verify inputs
  if (!verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name))
  {
    return false;
  }

  // get joint array
  if (!fill_joints_if_missing(joint_pos, q_.data))
  {
    return false;
  }

  // reset transform_vec
  transform.setIdentity();

  // special case: since the root is not in the robot tree, need to return identity transform
  if (link_name == root_name_)
  {
    return true;
  }

  // create forward kinematics solver
  fk_pos_solver_->JntToCart(q_, frame_, link_name_map_[link_name]);
  tf2::transformKDLToEigen(frame_, transform);
  return true;
}

bool DynamicsInterfaceFd::calculate_jacobian(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos, const std::string & link_name,
  Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian)
{
  // verify inputs
  if (
    !verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name) ||
    !verify_jacobian(jacobian))
  {
    return false;
  }

  // get joint array
  if (!fill_joints_if_missing(joint_pos, q_.data))
  {
    return false;
  };

  // calculate Jacobian
  jac_solver_->JntToJac(q_, *jacobian_, link_name_map_[link_name]);
  jacobian = jacobian_->data;

  return true;
}

bool DynamicsInterfaceFd::calculate_jacobian_derivative(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos,
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_vel, const std::string & link_name,
  Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian_derivative)
{
  // verify inputs
  if (
    !verify_initialized() || !verify_joint_vector(joint_pos) || !verify_joint_vector(joint_vel) ||
    !verify_link_name(link_name) || !verify_jacobian(jacobian_derivative))
  {
    return false;
  }

  // get joint array
  if (!fill_joints_if_missing(joint_pos, q_.data))
  {
    return false;
  };
  if (!fill_joints_if_missing(joint_vel, q_dot_.data))
  {
    return false;
  };

  q_array_vel_.q = q_;
  q_array_vel_.qdot = q_dot_;

  // calculate Jacobian
  jac_dot_solver_->JntToJacDot(q_array_vel_, *jacobian_derivative_, link_name_map_[link_name]);
  jacobian_derivative = jacobian_derivative_->data;

  return true;
}

// Dynamics
// --------------------------------------------------------------------

bool DynamicsInterfaceFd::calculate_inertia(
  const Eigen::VectorXd & joint_pos,
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & inertia)
{
  // verify inputs
  if (!verify_initialized() || !verify_joint_vector(joint_pos) || !verify_inertia(inertia))
  {
    return false;
  }

  // TODO

  return false;
}

bool DynamicsInterfaceFd::calculate_coriolis(
  const Eigen::VectorXd & joint_pos, const Eigen::VectorXd & joint_vel, Eigen::VectorXd & coriolis)
{
  // verify inputs
  if (
    !verify_initialized() || !verify_joint_vector(joint_pos) || !verify_joint_vector(joint_vel) ||
    !verify_coriolis(coriolis))
  {
    return false;
  }

  coriolis.setZero();

  return true;
}

bool DynamicsInterfaceFd::calculate_gravity(
  const Eigen::VectorXd & joint_pos, Eigen::VectorXd & gravity)
{
  // verify inputs
  if (!verify_initialized() || !verify_joint_vector(joint_pos) || !verify_gravity(gravity))
  {
    return false;
  }

  gravity.setZero();

  return true;
}

// Kinematics
// --------------------------------------------------------------------

bool DynamicsInterfaceFd::convert_joint_deltas_to_cartesian_deltas(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos,
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & delta_theta, const std::string & link_name,
  Eigen::Matrix<double, 6, 1> & delta_x)
{
  // verify inputs
  if (
    !verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name) ||
    !verify_joint_vector(delta_theta))
  {
    return false;
  }

  // get joint array
  if (!fill_joints_if_missing(joint_pos, q_.data))
  {
    return false;
  };

  // calculate Jacobian
  jac_solver_->JntToJac(q_, *jacobian_, link_name_map_[link_name]);
  delta_x = jacobian_->data * delta_theta;

  return true;
}

bool DynamicsInterfaceFd::convert_cartesian_deltas_to_joint_deltas(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos,
  const Eigen::Matrix<double, 6, 1> & delta_x, const std::string & link_name,
  Eigen::Matrix<double, Eigen::Dynamic, 1> & delta_theta)
{
  // verify inputs
  if (
    !verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name) ||
    !verify_joint_vector(delta_theta))
  {
    return false;
  }

  // get joint array
  if (!fill_joints_if_missing(joint_pos, q_.data))
  {
    return false;
  };

  // calculate Jacobian
  jac_solver_->JntToJac(q_, *jacobian_, link_name_map_[link_name]);
  // TODO(anyone): this dynamic allocation needs to be replaced
  Eigen::Matrix<double, 6, Eigen::Dynamic> J = jacobian_->data;
  // damped inverse
  Eigen::Matrix<double, Eigen::Dynamic, 6> J_inverse =
    (J.transpose() * J + alpha * I).inverse() * J.transpose();
  delta_theta = J_inverse * delta_x;

  return true;
}

// Verification functions
// --------------------------------------------------------------------

bool DynamicsInterfaceFd::verify_link_name(const std::string & link_name)
{
  if (link_name == root_name_)
  {
    return true;
  }
  if (link_name_map_.find(link_name) == link_name_map_.end())
  {
    std::string links;
    for (size_t i = 0; i < chain_.getNrOfSegments(); ++i)
    {
      links += "\n" + chain_.getSegment(i).getName();
    }
    RCLCPP_ERROR(
      LOGGER, "The link %s was not found in the robot chain. Available links are: %s",
      link_name.c_str(), links.c_str());
    return false;
  }
  return true;
}

bool DynamicsInterfaceFd::verify_joint_vector(const Eigen::VectorXd & joint_vector)
{
  if (static_cast<size_t>(joint_vector.size()) != num_joints_)
  {
    RCLCPP_ERROR(
      LOGGER, "Invalid joint vector size (%zu). Expected size is %zu.", joint_vector.size(),
      num_joints_);
    return false;
  }
  return true;
}

bool DynamicsInterfaceFd::verify_initialized()
{
  // check if interface is initialized
  if (!initialized)
  {
    RCLCPP_ERROR(
      LOGGER,
      "The KDL kinematics plugin was not initialized. Ensure you called the initialize method.");
    return false;
  }
  return true;
}

bool DynamicsInterfaceFd::verify_jacobian(
  const Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian)
{
  if (jacobian.rows() != jacobian_->rows() || jacobian.cols() != jacobian_->columns())
  {
    RCLCPP_ERROR(
      LOGGER, "The size of the jacobian (%zu, %zu) does not match the required size of (%u, %u)",
      jacobian.rows(), jacobian.cols(), jacobian_->rows(), jacobian_->columns());
    return false;
  }
  return true;
}

bool DynamicsInterfaceFd::verify_inertia(
  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & inertia)
{
  if (static_cast<size_t>(inertia.rows()) != num_joints_
  || static_cast<size_t>(inertia.cols()) != num_joints_)
  {
    RCLCPP_ERROR(
      LOGGER,
      "The size of the inertia matrix (%zu, %zu) does not match the required size of (%lu, %lu)",
      inertia.rows(), inertia.cols(), num_joints_, num_joints_);
    return false;
  }
  return true;
}

bool DynamicsInterfaceFd::verify_coriolis(const Eigen::VectorXd & coriolis)
{
  if (static_cast<size_t>(coriolis.size()) != num_joints_)
  {
    RCLCPP_ERROR(
      LOGGER, "The size of the coriolis vector (%zu) does not match the required size of (%lu)",
      coriolis.size(), num_joints_);
    return false;
  }
  return true;
}

bool DynamicsInterfaceFd::verify_gravity(const Eigen::VectorXd & gravity)
{
  if (static_cast<size_t>(gravity.size()) != num_joints_)
  {
    RCLCPP_ERROR(
      LOGGER, "The size of the gravity vector (%zu) does not match the required size of (%lu)",
      gravity.size(), num_joints_);
    return false;
  }

  return true;
}

}  // namespace dynamics_interface_fd

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dynamics_interface_fd::DynamicsInterfaceFd, dynamics_interface::DynamicsInterface)
PLUGINLIB_EXPORT_CLASS(
  dynamics_interface_fd::DynamicsInterfaceFd, kinematics_interface::KinematicsInterface)
