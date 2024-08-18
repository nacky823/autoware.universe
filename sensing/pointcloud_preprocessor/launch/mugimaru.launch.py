# SPDX-FileCopyrightText: 2024 nacky823 youjiyongmu4@gmail.com
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    param_file_name=LaunchConfiguration('param_file_name')
    declare_param_file_name=DeclareLaunchArgument(
        'param_file_name',
        default_value='iscas_museum.param.yaml',
    )
    param_path=PathJoinSubstitution([
        FindPackageShare('pointcloud_preprocessor'), 'config', param_file_name
    ])

    use_sim_time=LaunchConfiguration('use_sim_time')
    declare_use_sim_time=DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
    )

    param_list=[param_path, {'use_sim_time': use_sim_time}]

    back_crop_box_component=ComposableNode(
        package='pointcloud_preprocessor',
        plugin='pointcloud_preprocessor::CropBoxFilterComponent',
        name='back_crop_box_filter',
        parameters=param_list,
        remappings=[
            ("input", "livox/lidar"),
            ("output", "filtered_points/back_crop_box"),
        ],
    )
    down_crop_box_component=ComposableNode(
        package='pointcloud_preprocessor',
        plugin='pointcloud_preprocessor::CropBoxFilterComponent',
        name='down_crop_box_filter',
        parameters=param_list,
        remappings=[
            ("input", "filtered_points/back_crop_box"),
            ("output", "filtered_points/down_crop_box"),
        ],
    )
    voxel_grid_component=ComposableNode(
        package='pointcloud_preprocessor',
        plugin='pointcloud_preprocessor::VoxelGridDownsampleFilterComponent',
        name='voxel_grid_filter',
        parameters=param_list,
        remappings=[
            ("input", "filtered_points/down_crop_box"),
            ("output", "filtered_points/voxel_grid"),
        ],
    )
    random_downsample_component=ComposableNode(
        package='pointcloud_preprocessor',
        plugin='pointcloud_preprocessor::RandomDownsampleFilterComponent',
        name='random_downsample_filter',
        parameters=param_list,
        remappings=[
            ("input", "filtered_points/voxel_grid"),
            ("output", "filtered_points/random_downsample"),
        ],
    )
    distortion_corrector_component=ComposableNode(
        package='pointcloud_preprocessor',
        plugin='pointcloud_preprocessor::DistortionCorrectorComponent',
        name='distortion_corrector',
        parameters=param_list,
        remappings=[
            ("~/input/pointcloud", "filtered_points/random_downsample"),
            ("~/input/twist", "gyro_twist_with_covariance"),
            ("~/input/imu", "livox/imu"),
            ("~/output/pointcloud", "filtered_points/distortion_correct"),
        ],
    )

    container=ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='pointcloud_preprocessor_container',
        namespace='pointcloud_preprocessor',
        composable_node_descriptions=[
            back_crop_box_component,
            down_crop_box_component,
            voxel_grid_component,
            random_downsample_component,
            distortion_corrector_component,
        ],
        output='screen',
    )

    ld=LaunchDescription()

    ld.add_action(declare_param_file_name)
    ld.add_action(declare_use_sim_time)
    ld.add_action(container)

    return ld
