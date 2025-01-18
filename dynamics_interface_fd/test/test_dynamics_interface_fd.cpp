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
#include "std_msgs/msg/float64_multi_array.hpp"

#include "test_asset_omega6_description.hpp"

const size_t num_joints_omega6 = 6;
const size_t num_joints_omega3 = 3;

class TestDynamicsFdPlugin : public ::testing::Test
{
public:
  std::shared_ptr<pluginlib::ClassLoader<dynamics_interface::DynamicsInterface>> dyn_loader_;
  std::shared_ptr<dynamics_interface::DynamicsInterface> dyn_;

  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> kyn_loader_;
  std::shared_ptr<kinematics_interface::KinematicsInterface> kyn_;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::string end_effector_ = "fd_ee";

  const std::string robot_description_omega6 = ros2_control_test_assets::valid_omega6_urdf;
  const std::string robot_description_omega3 = ros2_control_test_assets::valid_omega3_urdf;

  std::string fd_inertia_topic_name = "/fd_inertia";
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mock_fd_inertia_publisher_;

  void SetUp()
  {
    // init ros
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
    std::string plugin_name = "dynamics_interface_fd/DynamicsInterfaceFd";

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

    // setup mock publisher
    mock_fd_inertia_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      fd_inertia_topic_name, rclcpp::SystemDefaultsQoS());
  }

  void publishInertia(Eigen::Matrix<double, 6, 6> inertia)
  {
    // wait for subscriber
    size_t wait_count = 0;
    while (node_->count_subscribers(fd_inertia_topic_name) == 0)
    {
      if (wait_count >= 5)
      {
        auto error_msg =
          std::string("publishing to ") + fd_inertia_topic_name + " but no node subscribes to it";
        throw std::runtime_error(error_msg);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      ++wait_count;
    }

    // publish
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(36);
    for (size_t i = 0; i < 6; i++)
    {
      for (size_t j = 0; j < 6; j++)
      {
        msg.data[i * 6 + j] = inertia(i, j);
      }
    }
    mock_fd_inertia_publisher_->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  void TearDown()
  {
    // shutdown ros
    rclcpp::shutdown();
  }

  void loadURDFParameter(const std::string & robot_description)
  {
    rclcpp::Parameter param_urdf("robot_description", robot_description);
    node_->declare_parameter("robot_description", "");
    node_->set_parameter(param_urdf);
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

  void loadEndEffectorNameParameter(const std::string & end_effector_name)
  {
    rclcpp::Parameter param_ee("dynamics.tip", end_effector_name);
    node_->declare_parameter("dynamics.tip", end_effector_name);
    node_->set_parameter(param_ee);
  }

  void loadAllParameters()
  {
    loadURDFParameter(robot_description_omega6);
    loadEndEffectorNameParameter(end_effector_);
    loadAlphaParameter();
    loadGravityParameter();
  }
};

TEST_F(TestDynamicsFdPlugin, FD_plugin_function_with_omega6_urdf)
{
  RCLCPP_INFO(node_->get_logger(), "Testing omega6 URDF...");

  // load robot description and alpha to parameter server
  loadAllParameters();

  // initialize the  plugin
  auto robot_param = rclcpp::Parameter();
  ASSERT_TRUE(node_->get_parameter("robot_description", robot_param));
  RCLCPP_INFO(node_->get_logger(), "Plugin instantiated successfully!");

  // publish inertia matrix
  Eigen::Matrix<double, 6, 6> mock_inertia = 2 * Eigen::Matrix<double, 6, 6>::Identity();
  publishInertia(mock_inertia);

  // dummy joint position and velocity
  Eigen::Matrix<double, Eigen::Dynamic, 1> pos =
    Eigen::Matrix<double, num_joints_omega6, 1>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> vel =
    Eigen::Matrix<double, num_joints_omega6, 1>::Zero();

  // calculate end effector transform
  Eigen::Isometry3d end_effector_transform;
  ASSERT_TRUE(dyn_->calculate_link_transform(pos, end_effector_, end_effector_transform));

  // calculate jacobian and its derivative
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian =
    Eigen::Matrix<double, 6, num_joints_omega6>::Zero();
  ASSERT_TRUE(dyn_->calculate_jacobian(pos, end_effector_, jacobian));

  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_dot =
    Eigen::Matrix<double, 6, num_joints_omega6>::Zero();
  ASSERT_TRUE(dyn_->calculate_jacobian_derivative(pos, vel, end_effector_, jacobian_dot));

  std::cout << "jacobien: \n" << jacobian << std::endl;
  std::cout << "jacobien_dot: \n" << jacobian_dot << std::endl;

  // calculate inertia, coriolis and gravity matrices
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> inertia =
    Eigen::Matrix<double, num_joints_omega6, num_joints_omega6>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> coriolis =
    Eigen::Matrix<double, num_joints_omega6, 1>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> gravity =
    Eigen::Matrix<double, num_joints_omega6, 1>::Zero();

  // test inertia
  ASSERT_TRUE(dyn_->calculate_inertia(pos, inertia));
  ASSERT_TRUE((mock_inertia - inertia).isMuchSmallerThan(0.001));
  std::cout << "inertia: \n" << inertia << std::endl;

  // test coriolis and gravity getters
  ASSERT_TRUE(dyn_->calculate_coriolis(pos, vel, coriolis));
  ASSERT_TRUE(dyn_->calculate_gravity(pos, gravity));

  // convert cartesian delta to joint delta
  Eigen::Matrix<double, 6, 1> delta_x = Eigen::Matrix<double, 6, 1>::Zero();
  delta_x[2] = 1;
  Eigen::Matrix<double, Eigen::Dynamic, 1> delta_theta =
    Eigen::Matrix<double, num_joints_omega6, 1>::Zero();
  ASSERT_TRUE(
    dyn_->convert_cartesian_deltas_to_joint_deltas(pos, delta_x, end_effector_, delta_theta));

  // convert joint delta to cartesian delta
  Eigen::Matrix<double, 6, 1> delta_x_est;
  ASSERT_TRUE(
    dyn_->convert_joint_deltas_to_cartesian_deltas(pos, delta_theta, end_effector_, delta_x_est));

  // Ensure kinematics math is correct
  for (size_t i = 0; i < static_cast<size_t>(delta_x.size()); ++i)
  {
    ASSERT_NEAR(delta_x[i], delta_x_est[i], 0.02);
  }
  RCLCPP_INFO(node_->get_logger(), "All good so far...");
}

TEST_F(TestDynamicsFdPlugin, FD_plugin_function_with_omega3_urdf)
{
  RCLCPP_INFO(node_->get_logger(), "Testing omega3 URDF...");

  // load robot description and alpha to parameter server
  loadURDFParameter(robot_description_omega3);
  loadEndEffectorNameParameter(end_effector_);
  loadAlphaParameter();
  loadGravityParameter();

  // initialize the  plugin
  auto robot_param = rclcpp::Parameter();
  ASSERT_TRUE(node_->get_parameter("robot_description", robot_param));
  RCLCPP_INFO(node_->get_logger(), "Plugin instantiated successfully!");

  // publish inertia matrix
  Eigen::Matrix<double, 6, 6> mock_inertia = 2 * Eigen::Matrix<double, 6, 6>::Identity();
  publishInertia(mock_inertia);

  // dummy joint position and velocity
  Eigen::Matrix<double, Eigen::Dynamic, 1> pos =
    Eigen::Matrix<double, num_joints_omega3, 1>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> vel =
    Eigen::Matrix<double, num_joints_omega3, 1>::Zero();

  // calculate end effector transform
  Eigen::Isometry3d end_effector_transform;
  ASSERT_TRUE(dyn_->calculate_link_transform(pos, end_effector_, end_effector_transform));

  // calculate jacobian and its derivative
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian =
    Eigen::Matrix<double, 6, num_joints_omega3>::Zero();
  ASSERT_TRUE(dyn_->calculate_jacobian(pos, end_effector_, jacobian));

  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_dot =
    Eigen::Matrix<double, 6, num_joints_omega3>::Zero();
  ASSERT_TRUE(dyn_->calculate_jacobian_derivative(pos, vel, end_effector_, jacobian_dot));

  std::cout << "jacobien: \n" << jacobian << std::endl;
  std::cout << "jacobien_dot: \n" << jacobian_dot << std::endl;

  // calculate inertia, coriolis and gravity matrices
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> inertia =
    Eigen::Matrix<double, num_joints_omega3, num_joints_omega3>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> coriolis =
    Eigen::Matrix<double, num_joints_omega3, 1>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> gravity =
    Eigen::Matrix<double, num_joints_omega3, 1>::Zero();

  // test inertia
  ASSERT_TRUE(dyn_->calculate_inertia(pos, inertia));
  ASSERT_TRUE((mock_inertia.block<3, 3>(0, 0) - inertia).isMuchSmallerThan(0.001));
  std::cout << "inertia: \n" << inertia << std::endl;

  // test coriolis and gravity getters
  ASSERT_TRUE(dyn_->calculate_coriolis(pos, vel, coriolis));
  ASSERT_TRUE(dyn_->calculate_gravity(pos, gravity));

  // convert cartesian delta to joint delta
  Eigen::Matrix<double, 6, 1> delta_x = Eigen::Matrix<double, 6, 1>::Zero();
  delta_x[2] = 1;
  Eigen::Matrix<double, Eigen::Dynamic, 1> delta_theta =
    Eigen::Matrix<double, num_joints_omega3, 1>::Zero();
  ASSERT_TRUE(
    dyn_->convert_cartesian_deltas_to_joint_deltas(pos, delta_x, end_effector_, delta_theta));

  // convert joint delta to cartesian delta
  Eigen::Matrix<double, 6, 1> delta_x_est;
  ASSERT_TRUE(
    dyn_->convert_joint_deltas_to_cartesian_deltas(pos, delta_theta, end_effector_, delta_x_est));

  // Ensure kinematics math is correct
  for (size_t i = 0; i < static_cast<size_t>(delta_x.size()); ++i)
  {
    ASSERT_NEAR(delta_x[i], delta_x_est[i], 0.02);
  }
}

TEST_F(TestDynamicsFdPlugin, FD_plugin_function_std_vector)
{
  // load robot description and alpha to parameter server
  loadAllParameters();

  // initialize the  plugin
  auto robot_param = rclcpp::Parameter();
  ASSERT_TRUE(node_->get_parameter("robot_description", robot_param));

  // publish inertia matrix
  Eigen::Matrix<double, 6, 6> mock_inertia = 2 * Eigen::Matrix<double, 6, 6>::Identity();
  publishInertia(mock_inertia);

  // calculate end effector transform
  std::vector<double> pos(num_joints_omega6, 0);
  std::vector<double> vel(num_joints_omega6, 0);
  Eigen::Isometry3d end_effector_transform;
  ASSERT_TRUE(dyn_->calculate_link_transform(pos, end_effector_, end_effector_transform));

  // calculate jacobian and its derivative
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian =
    Eigen::Matrix<double, 6, num_joints_omega6>::Zero();
  ASSERT_TRUE(dyn_->calculate_jacobian(pos, end_effector_, jacobian));

  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_dot =
    Eigen::Matrix<double, 6, num_joints_omega6>::Zero();
  ASSERT_TRUE(dyn_->calculate_jacobian_derivative(pos, vel, end_effector_, jacobian_dot));

  // calculate inertia, coriolis and gravity matrices
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> inertia =
    Eigen::Matrix<double, num_joints_omega6, num_joints_omega6>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> coriolis =
    Eigen::Matrix<double, num_joints_omega6, 1>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> gravity =
    Eigen::Matrix<double, num_joints_omega6, 1>::Zero();

  ASSERT_TRUE(dyn_->calculate_inertia(pos, inertia));
  ASSERT_TRUE(dyn_->calculate_coriolis(pos, vel, coriolis));
  ASSERT_TRUE(dyn_->calculate_gravity(pos, gravity));

  // convert cartesian delta to joint delta
  std::vector<double> delta_x = {0, 0, 0, 0, 0, 0};
  delta_x[2] = 1;
  std::vector<double> delta_theta(num_joints_omega6, 0);
  ASSERT_TRUE(
    dyn_->convert_cartesian_deltas_to_joint_deltas(pos, delta_x, end_effector_, delta_theta));

  // convert joint delta to cartesian delta
  std::vector<double> delta_x_est(6);
  ASSERT_TRUE(
    dyn_->convert_joint_deltas_to_cartesian_deltas(pos, delta_theta, end_effector_, delta_x_est));

  // Ensure kinematics math is correct
  for (size_t i = 0; i < static_cast<size_t>(delta_x.size()); ++i)
  {
    ASSERT_NEAR(delta_x[i], delta_x_est[i], 0.02);
  }
}

TEST_F(TestDynamicsFdPlugin, no_inertia_published)
{
  RCLCPP_INFO(node_->get_logger(), "Testing omega6 URDF...");

  // load robot description and alpha to parameter server
  loadAllParameters();

  // initialize the  plugin
  auto robot_param = rclcpp::Parameter();
  ASSERT_TRUE(node_->get_parameter("robot_description", robot_param));
  RCLCPP_INFO(node_->get_logger(), "Plugin instantiated successfully!");

  // dummy joint position and velocity
  Eigen::Matrix<double, Eigen::Dynamic, 1> pos =
    Eigen::Matrix<double, num_joints_omega6, 1>::Zero();

  // calculate inertia
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> inertia =
    Eigen::Matrix<double, num_joints_omega6, num_joints_omega6>::Zero();
  ASSERT_FALSE(dyn_->calculate_inertia(pos, inertia));
}

TEST_F(TestDynamicsFdPlugin, incorrect_input_sizes)
{
  // load robot description and alpha to parameter server
  loadAllParameters();

  auto robot_param = rclcpp::Parameter();
  ASSERT_TRUE(node_->get_parameter("robot_description", robot_param));
  auto robot_description_str = robot_param.as_string();

  ASSERT_TRUE(
    dyn_->initialize(robot_description_str, node_->get_node_parameters_interface(), "dynamics"));

  // define correct values
  Eigen::Matrix<double, Eigen::Dynamic, 1> pos =
    Eigen::Matrix<double, num_joints_omega6, 1>::Zero();
  Eigen::Isometry3d end_effector_transform;
  Eigen::Matrix<double, 6, 1> delta_x = Eigen::Matrix<double, 6, 1>::Zero();
  delta_x[2] = 1;
  Eigen::Matrix<double, Eigen::Dynamic, 1> delta_theta =
    Eigen::Matrix<double, num_joints_omega6, 1>::Zero();
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

TEST_F(TestDynamicsFdPlugin, FD_plugin_no_robot_description)
{
  // load alpha to parameter server
  loadAlphaParameter();
  loadEndEffectorNameParameter(end_effector_);
  loadGravityParameter();

  ASSERT_FALSE(dyn_->initialize("", node_->get_node_parameters_interface(), "dynamics"));
}

TEST_F(TestDynamicsFdPlugin, FD_plugin_no_tip)
{
  loadURDFParameter(robot_description_omega6);
  loadAlphaParameter();
  loadGravityParameter();

  auto robot_param = rclcpp::Parameter();
  ASSERT_TRUE(node_->get_parameter("robot_description", robot_param));
  ASSERT_FALSE(dyn_->initialize("", node_->get_node_parameters_interface(), "dynamics"));
}

TEST_F(TestDynamicsFdPlugin, FD_plugin_as_kinematics_interface_only)
{
  // load robot description and alpha to parameter server
  loadAllParameters();

  // initialize the  plugin

  // initialize the  plugin
  auto robot_param = rclcpp::Parameter();
  ASSERT_TRUE(node_->get_parameter("robot_description", robot_param));
  auto robot_description_str = robot_param.as_string();

  ASSERT_TRUE(
    kyn_->initialize(robot_description_str, node_->get_node_parameters_interface(), "dynamics"));

  // dummy joint position and velocity
  Eigen::Matrix<double, Eigen::Dynamic, 1> pos =
    Eigen::Matrix<double, num_joints_omega6, 1>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 1> vel =
    Eigen::Matrix<double, num_joints_omega6, 1>::Zero();

  // calculate end effector transform
  Eigen::Isometry3d end_effector_transform;
  ASSERT_TRUE(kyn_->calculate_link_transform(pos, end_effector_, end_effector_transform));

  // calculate jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian =
    Eigen::Matrix<double, 6, num_joints_omega6>::Zero();
  ASSERT_TRUE(kyn_->calculate_jacobian(pos, end_effector_, jacobian));

  // convert cartesian delta to joint delta
  Eigen::Matrix<double, 6, 1> delta_x = Eigen::Matrix<double, 6, 1>::Zero();
  delta_x[2] = 1;
  Eigen::Matrix<double, Eigen::Dynamic, 1> delta_theta =
    Eigen::Matrix<double, num_joints_omega6, 1>::Zero();
  ASSERT_TRUE(
    kyn_->convert_cartesian_deltas_to_joint_deltas(pos, delta_x, end_effector_, delta_theta));

  // convert joint delta to cartesian delta
  Eigen::Matrix<double, 6, 1> delta_x_est;
  ASSERT_TRUE(
    kyn_->convert_joint_deltas_to_cartesian_deltas(pos, delta_theta, end_effector_, delta_x_est));

  // Ensure kinematics math is correct
  for (size_t i = 0; i < static_cast<size_t>(delta_x.size()); ++i)
  {
    ASSERT_NEAR(delta_x[i], delta_x_est[i], 0.02);
  }
}
