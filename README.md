# dynamics_interface
ROS 2 package for using C++ robot dynamics in control applications

This package is based on the [ros-controls/kinematics_interface](https://github.com/ros-controls/kinematics_interface) package and add the following features useful for force or impedance control:
- compute the joint inertia matrix H using `calculate_inertia`
- co

Given a robot with $n$ joints, the following notations are considered :
$$
\begin{align}
  H \ddot{q} + C(q, \dot{q}) \dot{q} + G(q) = \tau_c + J(q)^T f_{ext}
\end{align}
$$
where $H \in \mathbb{R}^{n \times n}$ is the inertia matrix, $C \in \mathbb{R}^{n \times n}$ is the Coriolis matrix, and $G(q) \mathbb{R}^{n}$ is the column vector containing the contribution of gravity.
The (geometrical) Jacobian matrix is denoted $J(q) \mathbb{R}^{6 \times n}$ and the external wrench is $f_{ext} \mathbb{R}^{6}$.
Note that this sign convention consider generalized external wrench, hence NOT the wrench applied onto the robot (i.e., $f_{ext} = - f_{sensor}$, where $f_{sensor}$ is the external wrench measurement from a f/t sensor).