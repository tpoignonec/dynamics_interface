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

#ifndef DYNAMICS_INTERFACE_FD__DYNAMICS_INTERFACE_FD_HPP_
#define DYNAMICS_INTERFACE_FD__DYNAMICS_INTERFACE_FD_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "dynamics_interface_kdl/dynamics_interface_kdl.hpp"


#include <rclcpp/rclcpp.hpp>
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

namespace dynamics_interface_fd
{
class DynamicsInterfaceFd : public dynamics_interface_kdl::DynamicsInterfaceKDL
{
public:
  DynamicsInterfaceFd();

  virtual ~DynamicsInterfaceFd();

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

  bool calculate_inertia(
    const Eigen::VectorXd & joint_pos,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & inertia) override;

  bool calculate_coriolis(
    const Eigen::VectorXd & joint_pos, const Eigen::VectorXd & joint_vel,
    Eigen::VectorXd & coriolis) override;

  bool calculate_gravity(const Eigen::VectorXd & joint_pos, Eigen::VectorXd & gravity) override;

protected:
  // methods
  bool fill_joints_if_missing(
    const Eigen::VectorXd & joint_values, Eigen::VectorXd & joint_values_filled);

  std::string fd_inertia_topic_name_;

  // async node for inertia
  rclcpp::Node::SharedPtr async_node_;
  std::unique_ptr<std::thread> node_thread_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr fd_inertia_subscriber_ptr_;
  std::shared_ptr<std_msgs::msg::Float64MultiArray> fd_inertia_msg_;
  Eigen::Matrix<double, 6, 6> fd_inertia_;

  // Incoming fd_inertia data
  realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64MultiArray>>
  rt_fd_inertia_subscriber_ptr_;
};

}  // namespace dynamics_interface_fd

#endif  // DYNAMICS_INTERFACE_FD__DYNAMICS_INTERFACE_FD_HPP_
