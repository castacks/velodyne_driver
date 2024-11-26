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

VELO_1_NS = 'velodyne_1'
VELO_2_NS = 'velodyne_2'

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():

    # Velodyne 1
    driver_params_file = get_share_file(
        'velodyne_driver', 'config/yamaha_velodyne_params.yaml')
    velodyne_driver_node_1 = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        namespace=VELO_1_NS,
        output='both',
        parameters=[driver_params_file]
    )

    convert_params_file = get_share_file(
        'velodyne_pointcloud', 'config/VLP32C-velodyne_transform_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = get_share_file(
        'velodyne_pointcloud', 'params/VeloView-VLP-32C.yaml')
    velodyne_transform_node_1 = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        namespace=VELO_1_NS,
        output='both',
        parameters=[convert_params]
    )

    laserscan_params_file = get_share_file(
        'velodyne_laserscan', 'config/default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node_1 = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        namespace=VELO_1_NS,
        output='both',
        parameters=[laserscan_params_file]
    )

    # Velodyne 2
    velodyne_driver_node_2 = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        namespace=VELO_2_NS,
        output='both',
        parameters=[driver_params_file]
    )

    velodyne_transform_node_2 = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        namespace=VELO_2_NS,
        output='both',
        parameters=[convert_params]
    )

    velodyne_laserscan_node_2 = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        namespace=VELO_2_NS,
        output='both',
        parameters=[laserscan_params_file]
    )

    return launch.LaunchDescription([velodyne_driver_node_1,
                                     velodyne_transform_node_1,
                                     velodyne_laserscan_node_1,
                                     velodyne_driver_node_2,
                                     velodyne_transform_node_2,
                                     velodyne_laserscan_node_2,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=velodyne_driver_node_1,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=velodyne_driver_node_2,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])
