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
        FindPackageShare('ekf_localizer'), 'config', param_file_name
    ])

    use_sim_time=LaunchConfiguration('use_sim_time')
    declare_use_sim_time=DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
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

    ld.add_action(declare_param_file_name)
    ld.add_action(declare_use_sim_time)
    ld.add_action(ekf_localizer)

    return ld
