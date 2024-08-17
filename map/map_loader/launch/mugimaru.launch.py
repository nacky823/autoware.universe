# SPDX-FileCopyrightText: 2024 nacky823 youjiyongmu4@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    default_map_path=os.path.join(
        os.path.expanduser('~'), 'maps', 'iscas_museum', 'iscas_museum_3d.pcd'
    )
    map_path=LaunchConfiguration('map_path')
    declare_map_path=DeclareLaunchArgument(
        'map_path',
        default_value=default_map_path,
    )

    param_file_name=LaunchConfiguration('param_file_name')
    declare_param_file_name=DeclareLaunchArgument(
        'param_file_name',
        default_value='iscas_museum.param.yaml',
    )
    param_path=os.path.join(
        get_package_share_directory('map_loader'),
        'config', param_file_name
    )

    use_sim_time=LaunchConfiguration('use_sim_time')
    declare_use_sim_time=DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
    )

    map_loader=Node(
        package='map_loader',
        executable='pointcloud_map_loader',
        name='pointcloud_map_loader',
        parameters=[
            param_path,
            {'pcd_paths_or_directory': [map_path]},
            {'pcd_metadata_path': map_path},
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('output/pointcloud_map', 'map/pointcloud_map'),
            ('service/get_partial_pcd_map', 'map/get_partial_pointcloud_map'),
            ('service/get_selected_pcd_map', 'map/get_selected_pointcloud_map'),
        ],
        output='screen',
    )

    ld=LaunchDescription()

    ld.add_action(declare_map_path)
    ld.add_action(declare_param_file_name)
    ld.add_action(declare_use_sim_time)
    ld.add_action(map_loader)

    return ld
