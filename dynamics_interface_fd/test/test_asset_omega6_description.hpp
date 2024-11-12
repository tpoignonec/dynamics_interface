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

#ifndef TEST_ASSET_OMEGA6_DESCRIPTION_HPP_
#define TEST_ASSET_OMEGA6_DESCRIPTION_HPP_

namespace ros2_control_test_assets
{

const auto valid_omega6_urdf =
  R"(
    <?xml version="1.0" encoding='utf-8'?>
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="fd">
        <link name="fd_base"/>
        <link name="fd_x_link"/>
        <link name="fd_y_link"/>
        <link name="fd_z_link"/>
        <link name="fd_roll_link"/>
        <link name="fd_pitch_link"/>
        <link name="fd_yaw_link"/>
        <link name="fd_ee"/>

        <!-- Translation -->
        <joint name="fd_x" type="prismatic">
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <parent link="fd_base"/>
            <child link="fd_x_link"/>
            <axis xyz="1 0 0"/>
            <limit effort="12" velocity="1.0" lower="-2" upper="2" />
        </joint>
        <joint name="fd_y" type="prismatic">
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <parent link="fd_x_link"/>
            <child link="fd_y_link"/>
            <axis xyz="0 1 0"/>
            <limit effort="12" velocity="1.0" lower="-2" upper="2" />
        </joint>
        <joint name="fd_z" type="prismatic">
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <parent link="fd_y_link"/>
            <child link="fd_z_link"/>
            <axis xyz="0 0 1"/>
            <limit effort="12" velocity="1.0" lower="-2" upper="2" />
        </joint>

        <!-- Orientation from XYZ roll-pitch-yaw angles -->
        <joint name="fd_roll" type="revolute">
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <parent link="fd_z_link"/>
            <child link="fd_roll_link"/>
            <axis xyz="1 0 0"/>
            <limit effort="0.4" velocity="10.0" lower="-3.1415" upper="3.1415" />
        </joint>
        <joint name="fd_pitch" type="revolute">
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <parent link="fd_roll_link"/>
            <child link="fd_pitch_link"/>
            <axis xyz="0 1 0"/>
            <limit effort="0.4" velocity="10.0" lower="-3.1415" upper="3.1415" />
        </joint>
        <joint name="fd_yaw" type="revolute">
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <parent link="fd_pitch_link"/>
            <child link="fd_yaw_link"/>
            <axis xyz="0 0 1"/>
            <limit effort="0.4" velocity="10.0" lower="-3.1415" upper="3.1415" />
        </joint>

        <!-- Tool -->
        <joint name="fd_tool" type="fixed">
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <parent link="fd_yaw_link"/>
            <child link="fd_ee"/>
        </joint>

        <!-- Clutch -->
        <link name="fd_virtual_clutch_link"/>
        <joint name="fd_clutch" type="prismatic">
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <parent link="fd_ee"/>
            <child link="fd_virtual_clutch_link"/>
            <axis xyz="1 0 0"/>
            <limit effort="12" velocity="1.0" lower="-0.05" upper="0.05" />
        </joint>
    </robot>
)";

const auto valid_omega3_urdf =
  R"(
    <?xml version="1.0" encoding='utf-8'?>
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="fd">
        <link name="fd_base"/>
        <link name="fd_x_link"/>
        <link name="fd_y_link"/>
        <link name="fd_z_link"/>
        <link name="fd_roll_link"/>
        <link name="fd_pitch_link"/>
        <link name="fd_yaw_link"/>
        <link name="fd_ee"/>

        <!-- Translation -->
        <joint name="fd_x" type="prismatic">
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <parent link="fd_base"/>
            <child link="fd_x_link"/>
            <axis xyz="1 0 0"/>
            <limit effort="12" velocity="1.0" lower="-2" upper="2" />
        </joint>
        <joint name="fd_y" type="prismatic">
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <parent link="fd_x_link"/>
            <child link="fd_y_link"/>
            <axis xyz="0 1 0"/>
            <limit effort="12" velocity="1.0" lower="-2" upper="2" />
        </joint>
        <joint name="fd_z" type="prismatic">
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <parent link="fd_y_link"/>
            <child link="fd_z_link"/>
            <axis xyz="0 0 1"/>
            <limit effort="12" velocity="1.0" lower="-2" upper="2" />
        </joint>

        <!-- Orientation from XYZ roll-pitch-yaw angles -->
        <joint name="fd_roll" type="fixed">
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <parent link="fd_z_link"/>
            <child link="fd_roll_link"/>
            <axis xyz="1 0 0"/>
        </joint>
        <joint name="fd_pitch" type="fixed">
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <parent link="fd_roll_link"/>
            <child link="fd_pitch_link"/>
            <axis xyz="0 1 0"/>
        </joint>
        <joint name="fd_yaw" type="fixed">
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <parent link="fd_pitch_link"/>
            <child link="fd_yaw_link"/>
            <axis xyz="0 0 1"/>
        </joint>

        <!-- Tool -->
        <joint name="fd_tool" type="fixed">
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <parent link="fd_yaw_link"/>
            <child link="fd_ee"/>
        </joint>

        <!-- Clutch -->
        <link name="fd_virtual_clutch_link"/>
        <joint name="fd_clutch" type="prismatic">
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <parent link="fd_ee"/>
            <child link="fd_virtual_clutch_link"/>
            <axis xyz="1 0 0"/>
            <limit effort="12" velocity="1.0" lower="-0.05" upper="0.05" />
        </joint>
    </robot>
)";

}  // namespace ros2_control_test_assets

#endif  // TEST_ASSET_OMEGA6_DESCRIPTION_HPP_
