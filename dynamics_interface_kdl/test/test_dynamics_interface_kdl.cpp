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

#include <gmock/gmock.h>
#include <memory>
#include "dynamics_interface/dynamics_interface.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestKDLPlugin : public ::testing::Test
{
public:
  std::shared_ptr<pluginlib::ClassLoader<dynamics_interface::DynamicsInterface>> dyn_loader_;
  std::shared_ptr<dynamics_interface::DynamicsInterface> dyn_;

  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> kyn_loader_;
  std::shared_ptr<kinematics_interface::KinematicsInterface> kyn_;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::string end_effector_ = "link2";

  void SetUp()
  {
    // init ros
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
    std::string plugin_name = "dynamics_interface_kdl/DynamicsInterfaceKDL";

    // setup dynamics plugin instance
    dyn_loader_ = std::make_shared<pluginlib::ClassLoader<dynamics_interface::DynamicsInterface>>(
      "dynamics_interface", "dynamics_interface::DynamicsInterface");
    dyn_ = std::unique_ptr<dynamics_interface::DynamicsInterface>(
      dyn_loader_->createUnmanagedInstance(plugin_name));

    // setup kinematics plugin instance
    kyn_loader_ =
      std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
        "kinematics_interface", "kinematics_interface::KinematicsInterface");
    kyn_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
      kyn_loader_->createUnmanagedInstance(plugin_name));
  }

  void TearDown()
  {
    // shutdown ros
    rclcpp::shutdown();
  }

  void loadURDFParameter()
  {
    auto urdf = std::string(ros2_control_test_assets::urdf_head) +
      std::string(ros2_control_test_assets::urdf_tail);

    rclcpp::Parameter param_urdf("robot_description", urdf);
    node_->declare_parameter("robot_description", "");
    node_->set_parameter(param_urdf);
  }

  void loadEndEffectorNameParameter(const std::string & end_effector_name)
  {
    rclcpp::Parameter param_ee("dynamics.tip", end_effector_name);
    node_->declare_parameter("dynamics.tip", end_effector_name);
    node_->set_parameter(param_ee);
  }

  void loadAlphaParameter()
  {
    rclcpp::Parameter param("dynamics.alpha", 0.005);
    node_->declare_parameter("dynamics.alpha", 0.005);
    node_->set_parameter(param);
  }

  void loadGravityParameter()
  {
    std::vector<double> gravity = {0, 0, -9.81};
    rclcpp::Parameter param_gravity("dynamics.gravity", gravity);
    node_->declare_parameter("dynamics.gravity", gravity);
    node_->set_parameter(param_gravity);
  }

  void loadAllParameters()
  {
    loadURDFParameter();
    loadEndEffectorNameParameter(end_effector_);
    loadAlphaParameter();
    loadGravityParameter();
  }
};

TEST_F(TestKDLPlugin, KDL_plugin_function)
{
  // load robot description and alpha to parameter server
  loadAllParameters();

  // initialize the  plugin
  auto robot_param = rclcpp::Parameter();
  ASSERT_TRUE(node_->get_parameter("robot_description", robot_param));
  auto robot_description_str = robot_param.as_string();

  ASSERT_TRUE(
    dyn_->initialize(robot_description_str, node_->get_node_parameters_interface(), "dynamics"));

  // dummy joint position and velocity
  Eigen::Matrix<double, Eigen::Dynamic, 1> pos = Eigen::Matrix<double, 2, 1>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> vel = Eigen::Matrix<double, 2, 1>::Zero();

  // calculate end effector transform
  Eigen::Isometry3d end_effector_transform;
  ASSERT_TRUE(dyn_->calculate_link_transform(pos, end_effector_, end_effector_transform));

  // calculate jacobian and its derivative
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = Eigen::Matrix<double, 6, 2>::Zero();
  ASSERT_TRUE(dyn_->calculate_jacobian(pos, end_effector_, jacobian));

  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_dot = Eigen::Matrix<double, 6, 2>::Zero();
  ASSERT_TRUE(dyn_->calculate_jacobian_derivative(pos, vel, end_effector_, jacobian_dot));

  // calculate inertia, coriolis and gravity matrices
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> inertia =
    Eigen::Matrix<double, 2, 2>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> coriolis = Eigen::Matrix<double, 2, 1>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> gravity = Eigen::Matrix<double, 2, 1>::Zero();

  ASSERT_TRUE(dyn_->calculate_inertia(pos, inertia));
  ASSERT_TRUE(dyn_->calculate_coriolis(pos, vel, coriolis));
  ASSERT_TRUE(dyn_->calculate_gravity(pos, gravity));

  // convert cartesian delta to joint delta
  Eigen::Matrix<double, 6, 1> delta_x = Eigen::Matrix<double, 6, 1>::Zero();
  delta_x[2] = 1;
  Eigen::Matrix<double, Eigen::Dynamic, 1> delta_theta = Eigen::Matrix<double, 2, 1>::Zero();
  ASSERT_TRUE(
    dyn_->convert_cartesian_deltas_to_joint_deltas(pos, delta_x, end_effector_, delta_theta));

  // convert joint delta to cartesian delta
  Eigen::Matrix<double, 6, 1> delta_x_est;
  ASSERT_TRUE(
    dyn_->convert_joint_deltas_to_cartesian_deltas(pos, delta_theta, end_effector_, delta_x_est));

  // Ensure kinematics math is correct
  for (size_t i = 0; i < static_cast<size_t>(delta_x.size()); ++i) {
    ASSERT_NEAR(delta_x[i], delta_x_est[i], 0.02);
  }
}

TEST_F(TestKDLPlugin, KDL_plugin_function_std_vector)
{
  // load robot description and alpha to parameter server
  loadAllParameters();

  // initialize the  plugin
  auto robot_param = rclcpp::Parameter();
  ASSERT_TRUE(node_->get_parameter("robot_description", robot_param));

  auto robot_description_str = robot_param.as_string();
  ASSERT_TRUE(
    dyn_->initialize(robot_description_str, node_->get_node_parameters_interface(), "dynamics"));

  // calculate end effector transform
  std::vector<double> pos = {0, 0};
  std::vector<double> vel = {0, 0};
  Eigen::Isometry3d end_effector_transform;
  ASSERT_TRUE(dyn_->calculate_link_transform(pos, end_effector_, end_effector_transform));

  // calculate jacobian and its derivative
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = Eigen::Matrix<double, 6, 2>::Zero();
  ASSERT_TRUE(dyn_->calculate_jacobian(pos, end_effector_, jacobian));

  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_dot = Eigen::Matrix<double, 6, 2>::Zero();
  ASSERT_TRUE(dyn_->calculate_jacobian_derivative(pos, vel, end_effector_, jacobian_dot));

  // calculate inertia, coriolis and gravity matrices
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> inertia =
    Eigen::Matrix<double, 2, 2>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> coriolis = Eigen::Matrix<double, 2, 1>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> gravity = Eigen::Matrix<double, 2, 1>::Zero();

  ASSERT_TRUE(dyn_->calculate_inertia(pos, inertia));
  ASSERT_TRUE(dyn_->calculate_coriolis(pos, vel, coriolis));
  ASSERT_TRUE(dyn_->calculate_gravity(pos, gravity));

  // convert cartesian delta to joint delta
  std::vector<double> delta_x = {0, 0, 0, 0, 0, 0};
  delta_x[2] = 1;
  std::vector<double> delta_theta = {0, 0};
  ASSERT_TRUE(
    dyn_->convert_cartesian_deltas_to_joint_deltas(pos, delta_x, end_effector_, delta_theta));

  // convert joint delta to cartesian delta
  std::vector<double> delta_x_est(6);
  ASSERT_TRUE(
    dyn_->convert_joint_deltas_to_cartesian_deltas(pos, delta_theta, end_effector_, delta_x_est));

  // Ensure kinematics math is correct
  for (size_t i = 0; i < static_cast<size_t>(delta_x.size()); ++i) {
    ASSERT_NEAR(delta_x[i], delta_x_est[i], 0.02);
  }
}

TEST_F(TestKDLPlugin, incorrect_input_sizes)
{
  // load robot description and alpha to parameter server
  loadAllParameters();

  // initialize the  plugin
  auto robot_param = rclcpp::Parameter();
  ASSERT_TRUE(node_->get_parameter("robot_description", robot_param));
  auto robot_description_str = robot_param.as_string();

  ASSERT_TRUE(
    dyn_->initialize(robot_description_str, node_->get_node_parameters_interface(), "dynamics"));

  // define correct values
  Eigen::Matrix<double, Eigen::Dynamic, 1> pos = Eigen::Matrix<double, 2, 1>::Zero();
  Eigen::Isometry3d end_effector_transform;
  Eigen::Matrix<double, 6, 1> delta_x = Eigen::Matrix<double, 6, 1>::Zero();
  delta_x[2] = 1;
  Eigen::Matrix<double, Eigen::Dynamic, 1> delta_theta = Eigen::Matrix<double, 2, 1>::Zero();
  Eigen::Matrix<double, 6, 1> delta_x_est;

  // wrong size input vector
  Eigen::Matrix<double, Eigen::Dynamic, 1> vec_5 = Eigen::Matrix<double, 5, 1>::Zero();

  // calculate transform
  ASSERT_FALSE(dyn_->calculate_link_transform(vec_5, end_effector_, end_effector_transform));
  ASSERT_FALSE(dyn_->calculate_link_transform(pos, "link_not_in_model", end_effector_transform));

  // convert cartesian delta to joint delta
  ASSERT_FALSE(
    dyn_->convert_cartesian_deltas_to_joint_deltas(vec_5, delta_x, end_effector_, delta_theta));
  ASSERT_FALSE(
    dyn_->convert_cartesian_deltas_to_joint_deltas(pos, delta_x, "link_not_in_model", delta_theta));
  ASSERT_FALSE(dyn_->convert_cartesian_deltas_to_joint_deltas(pos, delta_x, end_effector_, vec_5));

  // convert joint delta to cartesian delta
  ASSERT_FALSE(
    dyn_->convert_joint_deltas_to_cartesian_deltas(vec_5, delta_theta, end_effector_, delta_x_est));
  ASSERT_FALSE(
    dyn_->convert_joint_deltas_to_cartesian_deltas(pos, vec_5, end_effector_, delta_x_est));
  ASSERT_FALSE(dyn_->convert_joint_deltas_to_cartesian_deltas(
    pos, delta_theta, "link_not_in_model", delta_x_est));
}

TEST_F(TestKDLPlugin, KDL_plugin_no_robot_description)
{
  // load alpha to parameter server
  loadURDFParameter();
  loadAlphaParameter();
  loadEndEffectorNameParameter(end_effector_);
  loadGravityParameter();

  // initialize the  plugin
  auto robot_param = rclcpp::Parameter();
  ASSERT_TRUE(node_->get_parameter("robot_description", robot_param));
  ASSERT_TRUE(dyn_->initialize("", node_->get_node_parameters_interface(), "dynamics"));
}

TEST_F(TestKDLPlugin, FD_plugin_no_tip)
{
  loadURDFParameter();
  loadAlphaParameter();
  loadGravityParameter();

  // initialize the  plugin
  auto robot_param = rclcpp::Parameter();
  ASSERT_TRUE(node_->get_parameter("robot_description", robot_param));
  ASSERT_FALSE(dyn_->initialize("", node_->get_node_parameters_interface(), "dynamics"));
}

TEST_F(TestKDLPlugin, KDL_plugin_no_gravity)
{
  // load alpha to parameter server
  loadURDFParameter();
  loadEndEffectorNameParameter(end_effector_);
  loadAlphaParameter();

  // initialize the  plugin
  auto robot_param = rclcpp::Parameter();
  ASSERT_TRUE(node_->get_parameter("robot_description", robot_param));
  auto robot_description_str = robot_param.as_string();

  ASSERT_FALSE(
    dyn_->initialize(robot_description_str, node_->get_node_parameters_interface(), "dynamics"));
}

TEST_F(TestKDLPlugin, KDL_plugin_as_kinematics_interface_only)
{
  // load robot description and alpha to parameter server
  loadAllParameters();

  // initialize the  plugin
  auto robot_param = rclcpp::Parameter();
  ASSERT_TRUE(node_->get_parameter("robot_description", robot_param));
  auto robot_description_str = robot_param.as_string();
  ASSERT_TRUE(
    kyn_->initialize(robot_description_str, node_->get_node_parameters_interface(), "dynamics"));

  // dummy joint position and velocity
  Eigen::Matrix<double, Eigen::Dynamic, 1> pos = Eigen::Matrix<double, 2, 1>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> vel = Eigen::Matrix<double, 2, 1>::Zero();

  // calculate end effector transform
  Eigen::Isometry3d end_effector_transform;
  ASSERT_TRUE(kyn_->calculate_link_transform(pos, end_effector_, end_effector_transform));

  // calculate jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = Eigen::Matrix<double, 6, 2>::Zero();
  ASSERT_TRUE(kyn_->calculate_jacobian(pos, end_effector_, jacobian));


  // calculate jacobian inverse
  Eigen::Matrix<double, Eigen::Dynamic, 6> jacobian_inverse =
    jacobian.completeOrthogonalDecomposition().pseudoInverse();

  Eigen::Matrix<double, Eigen::Dynamic, 6> jacobian_inverse_est =
    Eigen::Matrix<double, 2, 6>::Zero();
  ASSERT_TRUE(kyn_->calculate_jacobian_inverse(pos, end_effector_, jacobian_inverse_est));

  // ensure jacobian inverse math is correct
  for (size_t i = 0; i < static_cast<size_t>(jacobian_inverse.rows()); ++i) {
    for (size_t j = 0; j < static_cast<size_t>(jacobian_inverse.cols()); ++j) {
      ASSERT_NEAR(jacobian_inverse(i, j), jacobian_inverse_est(i, j), 0.02);
    }
  }

  // convert cartesian delta to joint delta
  Eigen::Matrix<double, 6, 1> delta_x = Eigen::Matrix<double, 6, 1>::Zero();
  delta_x[2] = 1;
  Eigen::Matrix<double, Eigen::Dynamic, 1> delta_theta = Eigen::Matrix<double, 2, 1>::Zero();
  ASSERT_TRUE(
    kyn_->convert_cartesian_deltas_to_joint_deltas(pos, delta_x, end_effector_, delta_theta));

  // convert joint delta to cartesian delta
  Eigen::Matrix<double, 6, 1> delta_x_est;
  ASSERT_TRUE(
    kyn_->convert_joint_deltas_to_cartesian_deltas(pos, delta_theta, end_effector_, delta_x_est));

  // Ensure kinematics math is correct
  for (size_t i = 0; i < static_cast<size_t>(delta_x.size()); ++i) {
    ASSERT_NEAR(delta_x[i], delta_x_est[i], 0.02);
  }
}
