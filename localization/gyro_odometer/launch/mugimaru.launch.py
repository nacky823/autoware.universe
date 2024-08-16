# SPDX-FileCopyrightText: 2024 nacky823 youjiyongmu4@gmail.com
# SPDX-License-Identifier: Apache-2.0

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

    gyro_odometer=Node(
        package='gyro_odometer',
        executable='gyro_odometer',
        name='gyro_odometer',
        parameters=[{
            'output_frame': 'base_link',
            'message_timeout_sec': 0.2,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('vehicle/twist_with_covariance', 'mugimaru_twist'),
            ('imu', 'livox/imu'),
            ('twist_raw', 'gyro_twist_raw'),
            ('twist_with_covariance_raw', 'gyro_twist_with_covariance_raw'),
            ('twist', 'gyro_twist'),
            ('twist_with_covariance', 'gyro_twist_with_covariance'),
        ],
        output='screen',
    )

    ld=LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(gyro_odometer)

    return ld
