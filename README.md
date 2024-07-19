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


__TODO:__
  - compute Coriolis matrix? Does not seem trivial with KDL, but should be easy with [Pinocchio](https://github.com/stack-of-tasks/pinocchio).
  - manage redundancy here? For instance, with some function like `convert_cartesian_wrench_to_joint_torque` that implement null-space objectives under the hood.