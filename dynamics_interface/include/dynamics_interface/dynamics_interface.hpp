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

// Based on package "ros-controls/kinematics_interface", Copyright (c) 2022, PickNik, Inc.

#ifndef DYNAMICS_INTERFACE__DYNAMICS_INTERFACE_HPP_
#define DYNAMICS_INTERFACE__DYNAMICS_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/LU"
#include "kinematics_interface/kinematics_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

namespace dynamics_interface
{
class DynamicsInterface : public kinematics_interface::KinematicsInterface
{
public:
  DynamicsInterface() = default;

  virtual ~DynamicsInterface() = default;

  /**
   * \brief Calculates the joint inertia matrix H.
   * \param[in] joint_pos joint positions of the robot in radians
   * \param[out] inertia joint inertia matrix
   * \return true if successful
   */
  virtual bool calculate_inertia(
    const Eigen::VectorXd & joint_pos,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & inertia) = 0;

  /**
   * \brief Calculates the vector containing the Coriolis and centrifugal terms (i.e., C @ q_dot).
   * \param[in] joint_pos joint positions of the robot in radians
   * \param[in] joint_vel joint velocities of the robot in radians per second
   * \param[out] coriolis coriolis and centrifugal terms
   * \return true if successful
   */
  virtual bool calculate_coriolis(
    const Eigen::VectorXd & joint_pos, const Eigen::VectorXd & joint_vel,
    Eigen::VectorXd & coriolis) = 0;

  /**
   * \brief Calculates the gravity matrix G (i.e., external torques resulting from gravity).
   * \param[in] joint_pos joint positions of the robot in radians
   * \param[out] gravity gravity
   * \return true if successful
   */
  virtual bool calculate_gravity(const Eigen::VectorXd & joint_pos, Eigen::VectorXd & gravity) = 0;

  /**
   * \brief Calculates the time derivative of the jacobian for a specified link using provided joint positions.
   * \param[in] joint_pos joint positions of the robot in radians
   * \param[in] joint_vel joint velocities of the robot in radians per second
   * \param[in] link_name the name of the link to find the transform for
   * \param[out] jacobian_derivative Jacobian matrix of the specified link in column major format.
   * \return true if successful
   */
  virtual bool calculate_jacobian_derivative(
    const Eigen::VectorXd & joint_pos, const Eigen::VectorXd & joint_vel,
    const std::string & link_name,
    Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian_derivative) = 0;

  // non-virtual std::vector version of the interfaces

  bool calculate_inertia(
    const std::vector<double> & joint_pos_vec,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & inertia);

  bool calculate_coriolis(
    const std::vector<double> & joint_pos_vec, const std::vector<double> & joint_vel_vec,
    Eigen::VectorXd & coriolis);

  bool calculate_gravity(const std::vector<double> & joint_pos_vec, Eigen::VectorXd & gravity);

  bool calculate_jacobian_derivative(
    const std::vector<double> & joint_pos_vec, const std::vector<double> & joint_vel_vec,
    const std::string & link_name, Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian_derivative);
};

}  // namespace dynamics_interface

#endif  // DYNAMICS_INTERFACE__DYNAMICS_INTERFACE_HPP_
