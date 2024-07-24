# dynamics_interface
ROS 2 package for using C++ robot dynamics in control applications

This package is based on the [ros-controls/kinematics_interface](https://github.com/ros-controls/kinematics_interface) package and add the following features useful for force or impedance control:
- compute the joint inertia matrix $H(q)$ using `calculate_inertia`;
- compute the vector containing the Coriolis and centrifugal terms $C(q, \dot{q})$ using `calculate_coriolis`. Note that the so-called Coriolis matrix is NOT returned, but instead its dot product with $\dot{q}$;
- compute the gravity terms $G(q)$ using `calculate_gravity`;
- compute the time derivative of the Jacobian matrix $\dot{J}(q, \dot{q}) = \cfrac{d}{dt}J(q)$ using `calculate_derivative_jacobian`.

Given a robot with $n$ joints, the following notations are considered :

$$
\begin{align}
  H(q) \ddot{q} + C(q, \dot{q}) + G(q) = \tau_c + J(q)^T f_{ext}
\end{align}
$$

where $H \in \mathbb{R}^{n \times n}$ is the inertia matrix, $C \in \mathbb{R}^{n \times n}$ is the Coriolis matrix, and $G(q) \mathbb{R}^{n}$ is the column vector containing the contribution of gravity.
The (geometrical) Jacobian matrix is denoted $J(q) \mathbb{R}^{6 \times n}$ and the external wrench is $f_{ext} \mathbb{R}^{6}$.
Note that this sign convention consider the external wrench performing work on the cartesian velocity, hence NOT the wrench applied onto the robot... Typically, $f_{ext} = - f_{sensor}$, where $f_{sensor}$ is the external wrench measurement from a f/t sensor attached to the robot's end-effector.

## Implemented interfaces

### Generic KDL-based implementation

The package [dynamics_interface_kdl](dynamics_interface_kdl/src/dynamics_interface_kdl.cpp) is a specialization of the [kinematics_interface_kdl](https://github.com/ros-controls/kinematics_interface) package on which it is based.

### Custom interface for force dimension haptic interfaces

The package [dynamics_interface_fd](dynamics_interface_fd/src/dynamics_interface_fd.cpp) is intended to be used in conjunction with the stack [ICube-Robotics/forcedimension_ros2](https://github.com/ICube-Robotics/forcedimension_ros2).

Since the inertia cannot be computed from the URDF, a asynchronous node is instantiated to subscribe to the `fd_inertia` topic published by a [fd_inertia_broadcaster](https://github.com/ICube-Robotics/forcedimension_ros2/tree/main/fd_controllers/fd_inertia_broadcaster).
Note that both the coriolis and gravity vector are zero, because they are negligible on most devices (i.e., gravity compensated, no heavy part in rotation).

Typical `.yaml` configuration:

```yaml
dynamics_interface_fd:
  plugin_name: dynamics_interface_fd/DynamicsInterfaceFd
  plugin_package: dynamics_interface
  base: fd_base
  tip: fd_ee
  fd_inertia_topic_name: fd_inertia
  alpha: 0.0005
```

__TODO:__
  - compute Coriolis matrix? Does not seem trivial with KDL, but should be easy with [Pinocchio](https://github.com/stack-of-tasks/pinocchio).
  - manage redundancy here? For instance, with some function like `convert_cartesian_wrench_to_joint_torque` that implement null-space objectives under the hood.
