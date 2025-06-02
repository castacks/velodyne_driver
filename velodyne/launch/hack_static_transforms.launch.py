# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch the velodyne driver, pointcloud, and laserscan nodes with default configuration."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():

    vehicle_to_multisense = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='both',
        # arguments=["0", "0", "0", "0", "0", "0", "vehicle", "multisense/head"]
        arguments=["-0.09492452", "0.06414485", "-0.135983", "-0.01191501", "0.11306512", "0.000305", "0.99351609", "vehicle", "multisense/head"]
    )

    #need to add this to work on old hbag for now/mux out old static tf
    # temp_hack = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     output='both',
    #     arguments=["0.005", "0.105", "0.000", "-0.5", "0.5", "-0.5", "0.5", "multisense/head", "multisense/left_camera_optical_frame"]
    # )

    vehicle_to_velodyne_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='both',
        arguments=["0", "0", "0", "0", "0", "0", "vehicle", "velodyne_1"]
    )

    vehicle_to_velodyne_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='both',
        arguments=["-0.115", "0", "0.205", "0", "0.174533", "0", "vehicle", "velodyne_2"]
    )

    vehicle_to_novatel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='both',
        arguments=["-0.508", "-0.05", "-0.152", "0", "0", "0", "vehicle", "novatel/imu_frame"]
    )

    return launch.LaunchDescription([
        vehicle_to_multisense,
        # temp_hack,
        vehicle_to_velodyne_1,
        vehicle_to_velodyne_2,
        vehicle_to_novatel,
                                     ])
