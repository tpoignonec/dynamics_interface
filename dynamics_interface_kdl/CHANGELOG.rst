^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamics_interface_kdl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2025-18-01)
------------------
* Fix after kinematics_interface API break (PR #6)
* add `jacobian_inverse`method to track kinematics interface changes (see https://github.com/ros-controls/kinematics_interface/pull/92)
* fix broken tests
* Contributors: Thibault Poignonec

0.0.1 (2024-19-07)
------------------
* refractor kinematics_interface_kdl package
* remove "deta_to/from_..." functions from the dynamics_interface package
* add inertia, gravity, and coriolis matrices to the dynamics_interface package
* Contributors: Thibault Poignonec
