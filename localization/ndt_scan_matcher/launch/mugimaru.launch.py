# SPDX-FileCopyrightText: 2024 nacky823 youjiyongmu4@gmail.com
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    param_file_name=LaunchConfiguration('param_file_name')
    declare_param_file_name=DeclareLaunchArgument(
        'param_file_name',
        default_value='iscas_museum.param.yaml',
    )
    param_path=PathJoinSubstitution([
        FindPackageShare('ndt_scan_matcher'), 'config', param_file_name
    ])

    use_sim_time=LaunchConfiguration('use_sim_time')
    declare_use_sim_time=DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
    )

    ndt_scan_matcher=Node(
        package='ndt_scan_matcher',
        executable='ndt_scan_matcher',
        name='ndt_scan_matcher',
        parameters=[
            param_path,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('points_raw', 'filtered_points/distortion_correct'),
            ('ekf_pose_with_covariance', 'ekf_pose_with_covariance'),
            ('pointcloud_map', 'map/pointcloud_map'),
            ('regularization_pose_with_covariance', 'sensing/gnss/pose_with_covariance'),
            ('ekf_odom', 'ekf_odom'),
            ('trigger_node_srv', 'trigger_node'),
            ('pcd_loader_service', 'map/get_differential_pointcloud_map'),
            ('ndt_pose', 'ndt_pose'),
            ('ndt_pose_with_covariance', 'ndt_pose_with_covariance'),
        ],
        output='screen',
    )

    ld=LaunchDescription()

    ld.add_action(declare_param_file_name)
    ld.add_action(declare_use_sim_time)
    ld.add_action(ndt_scan_matcher)

    return ld
