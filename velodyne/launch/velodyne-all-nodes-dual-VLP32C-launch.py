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

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription()


    # velodyne 1
    driver_share_dir_1 = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    driver_params_file_1 = os.path.join(driver_share_dir_1, 'config', 'VLP32C-velodyne_driver_node-params_1.yaml')
    velodyne_driver_node_1 = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[driver_params_file_1])

    convert_share_dir_1 = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    convert_params_file_1 = os.path.join(convert_share_dir_1, 'config', 'VLP32C-velodyne_transform_node-params.yaml')
    with open(convert_params_file_1, 'r') as f:
        convert_params_1 = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params_1['calibration'] = os.path.join(convert_share_dir_1, 'params', 'VeloView-VLP-32C.yaml')
    velodyne_transform_node_1 = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                      executable='velodyne_transform_node',
                                                      output='both',
                                                      parameters=[convert_params_1])

    laserscan_share_dir_1 = ament_index_python.packages.get_package_share_directory('velodyne_laserscan')
    laserscan_params_file_1 = os.path.join(laserscan_share_dir_1, 'config', 'default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node_1 = launch_ros.actions.Node(package='velodyne_laserscan',
                                                      executable='velodyne_laserscan_node',
                                                      output='both',
                                                      parameters=[laserscan_params_file_1])

    # velodyne 2
    driver_share_dir_2 = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    driver_params_file_2 = os.path.join(driver_share_dir_2, 'config', 'VLP32C-velodyne_driver_node-params_2.yaml')
    velodyne_driver_node_2 = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[driver_params_file_2])

    convert_share_dir_2 = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    convert_params_file_2 = os.path.join(convert_share_dir_2, 'config', 'VLP32C-velodyne_transform_node-params.yaml')
    with open(convert_params_file_2, 'r') as f:
        convert_params_2 = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params_2['calibration'] = os.path.join(convert_share_dir_2, 'params', 'VeloView-VLP-32C.yaml')
    velodyne_transform_node_2 = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                      executable='velodyne_transform_node',
                                                      output='both',
                                                      parameters=[convert_params_2])

    laserscan_share_dir_2 = ament_index_python.packages.get_package_share_directory('velodyne_laserscan')
    laserscan_params_file_2 = os.path.join(laserscan_share_dir_2, 'config', 'default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node_2 = launch_ros.actions.Node(package='velodyne_laserscan',
                                                      executable='velodyne_laserscan_node',
                                                      output='both',
                                                      parameters=[laserscan_params_file_2])

    ld.add_action(velodyne_driver_node_1)
    ld.add_action(velodyne_transform_node_1)
    ld.add_action(velodyne_laserscan_node_1)
    ld.add_action(velodyne_driver_node_2)
    ld.add_action(velodyne_transform_node_2)
    ld.add_action(velodyne_laserscan_node_2)




    return ld
