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

#include <cstdlib>
#include <ctime>

#include "dynamics_interface_fd/dynamics_interface_fd.hpp"

namespace dynamics_interface_fd
{
rclcpp::Logger LOGGER = rclcpp::get_logger("dynamics_interface_fd");

bool fromMsg(const std_msgs::msg::Float64MultiArray & m, Eigen::Matrix<double, 6, 6> & e)
{
  int ii = 0;
  for (int i = 0; i < e.rows(); ++i) {
    for (int j = 0; j < e.cols(); ++j) {
      e(i, j) = m.data[ii++];
    }
  }
  return true;
}

DynamicsInterfaceFd::DynamicsInterfaceFd()
: DynamicsInterfaceKDL::DynamicsInterfaceKDL(),
  rt_fd_inertia_subscriber_ptr_(nullptr)
{
}

DynamicsInterfaceFd::~DynamicsInterfaceFd()
{
  RCLCPP_INFO(LOGGER, "Deactivating DynamicsInterfaceFd internal communication... please wait...");

  // shutdown node thread
  if (node_thread_ != nullptr) {
    executor_.cancel();
    node_thread_->join();
    node_thread_.reset();
    async_node_.reset();
  }
  // Call the destructor of the base class
  DynamicsInterfaceKDL::~DynamicsInterfaceKDL();
  RCLCPP_INFO(LOGGER, "Successfully deactivated!");
}

bool DynamicsInterfaceFd::initialize(
  const std::string & robot_description,
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
  const std::string & param_namespace)
{
  // track initialization plugin
  initialized = true;

  if (!DynamicsInterfaceKDL::initialize(robot_description, parameters_interface, param_namespace)) {
    RCLCPP_ERROR(LOGGER, "Failed to initialize DynamicsInterfaceKDL");
    return false;
  }

  // get parameters
  std::string ns = !param_namespace.empty() ? param_namespace + "." : "";

  // get inertia topic name
  auto fd_inertia_topic_name_param = rclcpp::Parameter();
  if (parameters_interface->has_parameter(ns + "fd_inertia_topic_name"))
  {
    parameters_interface->get_parameter(ns + "fd_inertia_topic_name", fd_inertia_topic_name_param);
    fd_inertia_topic_name_ = fd_inertia_topic_name_param.as_string();
  }
  else
  {
    fd_inertia_topic_name_ = "fd_inertia";
  }

  // Setup internal inertia subscriber
  RCLCPP_INFO(LOGGER, "Setting up internal inertia subscriber... please wait...");
  rclcpp::NodeOptions options;
  std::string node_name =
    "dynamic_interface_fd_internal_inertia_subscriber_" + std::to_string(std::rand());
  RCLCPP_INFO(LOGGER, "Internal inertia subscriber name: %s", node_name.c_str());
  options.arguments({"--ros-args", "-r", "__node:=" + node_name});
  async_node_ = rclcpp::Node::make_shared("_", options);

  fd_inertia_subscriber_ptr_ = async_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    fd_inertia_topic_name_, rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {rt_fd_inertia_subscriber_ptr_.writeFromNonRT(msg);});

  node_thread_ = std::make_unique<std::thread>(
    [&]()
    {
      executor_.add_node(async_node_);
      executor_.spin();
      executor_.remove_node(async_node_);
    });

  // TODO(tpoignonec): wait for subscriber to be set up?
  RCLCPP_INFO(LOGGER, "Subscriber node successfully set up!");

  return true;
}

bool DynamicsInterfaceFd::fill_joints_if_missing(
  const Eigen::VectorXd & joint_values, Eigen::VectorXd & joint_values_filled)
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

// Dynamics
// --------------------------------------------------------------------

bool DynamicsInterfaceFd::calculate_inertia(
  const Eigen::VectorXd & joint_pos,
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & inertia)
{
  // verify inputs
  if (!verify_initialized() || !verify_joint_vector(joint_pos) || !verify_inertia(inertia)) {
    return false;
  }

  // Read inertia matrix from rt subscriber
  fd_inertia_msg_ = *rt_fd_inertia_subscriber_ptr_.readFromRT();

  // if message exists, load values into references
  if (!fd_inertia_msg_.get()) {
    RCLCPP_ERROR(LOGGER, "No message received from fd_inertia real-time subscriber.");
    return false;
  }

  // check dimension of message
  if (fd_inertia_msg_->data.size() != static_cast<size_t>(36)) {
    RCLCPP_ERROR(
      LOGGER,
      "The size of the inertia matrix msg (%zu) does not match the required size of (6 x 6)",
      fd_inertia_msg_->data.size());
    return false;
  }
  fromMsg(*fd_inertia_msg_, fd_inertia_);

  // load values from message
  if (num_joints_ == 3) {
    inertia = fd_inertia_.block<3, 3>(0, 0);
  } else if (num_joints_ == 6) {
    inertia = fd_inertia_;
  } else {
    RCLCPP_ERROR(LOGGER, "Invalid number of joints (%lu).", num_joints_);
    return false;
  }
  return true;
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
  if (!verify_initialized() || !verify_joint_vector(joint_pos) || !verify_gravity(gravity)) {
    return false;
  }

  gravity.setZero();

  return true;
}

}  // namespace dynamics_interface_fd

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dynamics_interface_fd::DynamicsInterfaceFd, dynamics_interface::DynamicsInterface)
PLUGINLIB_EXPORT_CLASS(
  dynamics_interface_fd::DynamicsInterfaceFd, kinematics_interface::KinematicsInterface)
