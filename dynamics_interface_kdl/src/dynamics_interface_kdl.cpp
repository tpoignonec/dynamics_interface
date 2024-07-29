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

#include "dynamics_interface_kdl/dynamics_interface_kdl.hpp"

namespace dynamics_interface_kdl
{
rclcpp::Logger LOGGER = rclcpp::get_logger("dynamics_interface_kdl");

bool DynamicsInterfaceKDL::initialize(
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
  auto alpha_param = rclcpp::Parameter("dynamics.alpha", 0.000005);
  if (parameters_interface->has_parameter("dynamics.alpha"))
  {
    parameters_interface->get_parameter("dynamics.alpha", alpha_param);
  }
  alpha = alpha_param.as_double();

  // create kinematic chain
  KDL::Tree robot_tree;
  kdl_parser::treeFromString(robot_description, robot_tree);

  // get root name
  auto base_param = rclcpp::Parameter();
  if (parameters_interface->has_parameter("dynamics.base"))
  {
    parameters_interface->get_parameter("dynamics.base", base_param);
    root_name_ = base_param.as_string();
  }
  else
  {
    root_name_ = robot_tree.getRootSegment()->first;
  }

  // get gravity vector in base frame
  auto gravity_param = rclcpp::Parameter();
  if (parameters_interface->has_parameter("dynamics.gravity"))
  {
    parameters_interface->get_parameter("dynamics.gravity", gravity_param);
    std::vector<double> gravity_vec = gravity_param.as_double_array();
    if (gravity_vec.size() != 3)
    {
      RCLCPP_ERROR(
        LOGGER, "The size of the gravity vector (%zu) does not match the required size of 3",
        gravity_vec.size());
      return false;
    }
    gravity_in_base_frame_[0] = gravity_vec[0];
    gravity_in_base_frame_[1] = gravity_vec[1];
    gravity_in_base_frame_[2] = gravity_vec[2];
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Please specify a gravity vector in base frame '%s'.", root_name_.c_str());
    return false;
    /*
    // default gravity vector
    gravity_in_base_frame_[0] = 0.0;
    gravity_in_base_frame_[1] = 0.0;
    gravity_in_base_frame_[2] = -9.81;
    RCLCPP_INFO(
      LOGGER, "Gravity set to %f %f %f w.r.t. base frame '%s'.",
      gravity_in_base_frame_[0], gravity_in_base_frame_[1], gravity_in_base_frame_[2], root_name_.c_str());
    */
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
  inertia_ = std::make_shared<KDL::JntSpaceInertiaMatrix>(num_joints_);
  // coriolis_ = KDL::JntArray(num_joints_);
  // gravity_ = KDL::JntArray(num_joints_);

  // create KDL solvers
  fk_pos_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
  jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);
  jac_dot_solver_ = std::make_shared<KDL::ChainJntToJacDotSolver>(chain_);
  dyn_solver_ = std::make_shared<KDL::ChainDynParam>(chain_, gravity_in_base_frame_);

  return true;
}

// FK, Jacobian, and Jacobian time derivative
// --------------------------------------------------------------------

bool DynamicsInterfaceKDL::calculate_link_transform(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos, const std::string & link_name,
  Eigen::Isometry3d & transform)
{
  // verify inputs
  if (!verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name))
  {
    return false;
  }

  // get joint array
  q_.data = joint_pos;

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

bool DynamicsInterfaceKDL::calculate_jacobian(
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
  q_.data = joint_pos;

  // calculate Jacobian
  jac_solver_->JntToJac(q_, *jacobian_, link_name_map_[link_name]);
  jacobian = jacobian_->data;

  return true;
}

bool DynamicsInterfaceKDL::calculate_jacobian_derivative(
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
  q_.data = joint_pos;
  q_dot_.data = joint_vel;

  q_array_vel_.q = q_;
  q_array_vel_.qdot = q_dot_;

  // calculate Jacobian
  jac_dot_solver_->JntToJacDot(q_array_vel_, *jacobian_derivative_, link_name_map_[link_name]);
  jacobian_derivative = jacobian_derivative_->data;

  return true;
}

// Dynamics
// --------------------------------------------------------------------

bool DynamicsInterfaceKDL::calculate_inertia(
  const Eigen::VectorXd & joint_pos,
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & inertia)
{
  // verify inputs
  if (!verify_initialized() || !verify_joint_vector(joint_pos) || !verify_inertia(inertia))
  {
    return false;
  }

  // get joint array
  q_.data = joint_pos;

  // calculate inertia
  dyn_solver_->JntToMass(q_, *inertia_);
  inertia = inertia_->data;

  return true;
}

bool DynamicsInterfaceKDL::calculate_coriolis(
  const Eigen::VectorXd & joint_pos, const Eigen::VectorXd & joint_vel, Eigen::VectorXd & coriolis)
{
  // verify inputs
  if (
    !verify_initialized() || !verify_joint_vector(joint_pos) || !verify_joint_vector(joint_vel) ||
    !verify_coriolis(coriolis))
  {
    return false;
  }

  // get joint array
  q_.data = joint_pos;
  q_dot_.data = joint_vel;

  // calculate coriolis
  dyn_solver_->JntToCoriolis(q_, q_dot_, coriolis_);
  coriolis = coriolis_.data;

  return true;
}

bool DynamicsInterfaceKDL::calculate_gravity(
  const Eigen::VectorXd & joint_pos, Eigen::VectorXd & gravity)
{
  // verify inputs
  if (!verify_initialized() || !verify_joint_vector(joint_pos) || !verify_gravity(gravity))
  {
    return false;
  }

  // get joint array
  q_.data = joint_pos;

  // calculate gravity
  dyn_solver_->JntToGravity(q_, gravity_);
  gravity = gravity_.data;

  return true;
}

// Kinematics
// --------------------------------------------------------------------

bool DynamicsInterfaceKDL::convert_joint_deltas_to_cartesian_deltas(
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
  q_.data = joint_pos;

  // calculate Jacobian
  jac_solver_->JntToJac(q_, *jacobian_, link_name_map_[link_name]);
  delta_x = jacobian_->data * delta_theta;

  return true;
}

bool DynamicsInterfaceKDL::convert_cartesian_deltas_to_joint_deltas(
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
  q_.data = joint_pos;

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

bool DynamicsInterfaceKDL::verify_link_name(const std::string & link_name)
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

bool DynamicsInterfaceKDL::verify_joint_vector(const Eigen::VectorXd & joint_vector)
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

bool DynamicsInterfaceKDL::verify_initialized()
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

bool DynamicsInterfaceKDL::verify_jacobian(
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

bool DynamicsInterfaceKDL::verify_inertia(
  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & inertia)
{
  if (inertia.rows() != inertia_->rows() || inertia.cols() != inertia_->columns())
  {
    RCLCPP_ERROR(
      LOGGER,
      "The size of the inertia matrix (%zu, %zu) does not match the required size of (%u, %u)",
      inertia.rows(), inertia.cols(), inertia_->rows(), inertia_->columns());
    return false;
  }
  return true;
}

bool DynamicsInterfaceKDL::verify_coriolis(const Eigen::VectorXd & coriolis)
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

bool DynamicsInterfaceKDL::verify_gravity(const Eigen::VectorXd & gravity)
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

}  // namespace dynamics_interface_kdl

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dynamics_interface_kdl::DynamicsInterfaceKDL, dynamics_interface::DynamicsInterface)
PLUGINLIB_EXPORT_CLASS(
  dynamics_interface_kdl::DynamicsInterfaceKDL, kinematics_interface::KinematicsInterface)
