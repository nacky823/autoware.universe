# SPDX-FileCopyrightText: 2024 nacky823 youjiyongmu4@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time=LaunchConfiguration('use_sim_time')
    declare_use_sim_time=DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
    )

    param_path=os.path.join(
        get_package_share_directory('ekf_localizer'),
        'config', 'mugimaru.param.yaml'
    )

    ekf_localizer=Node(
        package='ekf_localizer',
        executable='ekf_localizer',
        name='ekf_localizer',
        parameters=[
            param_path,
            {'pose_frame_id': 'map'},
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('in_pose_with_covariance', 'ndt_pose_with_covariance'),
            ('in_twist_with_covariance', 'gyro_twist_with_covariance'),
            ('initialpose', 'initialpose'),
            ('trigger_node_srv', 'trigger_node'),
            ('ekf_odom', 'ekf_odom'),
            ('ekf_pose', 'ekf_pose'),
            ('ekf_pose_with_covariance', 'ekf_pose_with_covariance'),
            ('ekf_biased_pose', 'ekf_biased_pose'),
            ('ekf_biased_pose_with_covariance', 'ekf_biased_pose_with_covariance'),
            ('ekf_twist', 'ekf_twist'),
            ('ekf_twist_with_covariance', 'ekf_twist_with_covariance'),
        ],
        output='screen',
    )

    ld=LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(ekf_localizer)

    return ld
