# SPDX-FileCopyrightText: 2024 nacky823 youjiyongmu4@gmail.com
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import SetParameter
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):

    set_use_sim_time = SetParameter(name="use_sim_time", value=True)

    return [set_use_sim_time]


def generate_launch_description():

    launch_setup_func = OpaqueFunction(function=launch_setup)

    ns = "pointcloud_preprocessor"
    pkg = "pointcloud_preprocessor"

    crop_box_component = ComposableNode(
        package = pkg,
        plugin = "pointcloud_preprocessor::CropBoxFilterComponent",
        name = "crop_box_filter",
        remappings = [
            ("input", "ray/pointcloud2"),
            ("output", "filtered_points/crop_box"),
        ],
        parameters = [
            {
                "input_frame": "base_link",
                "output_frame": "base_link",
                "min_x": -6.0,
                "max_x": 0.0,
                "min_y": -1.5,
                "max_y": 1.5,
                "min_z": -2.0,
                "max_z": 2.0,
                "negative": True,
            }
        ],
    )

    voxel_grid_component = ComposableNode(
        package = pkg,
        plugin = "pointcloud_preprocessor::VoxelGridDownsampleFilterComponent",
        name = "voxel_grid_filter",
        remappings = [
            ("input", "filtered_points/crop_box"),
            ("output", "filtered_points/voxel_grid"),
        ],
        parameters = [
            {
                "input_frame": "base_link",
                "output_frame": "base_link",
                "voxel_size_x": 0.3,
                "voxel_size_y": 0.3,
                "voxel_size_z": 0.3,
            }
        ],
    )

    random_downsample_component = ComposableNode(
        package = pkg,
        plugin = "pointcloud_preprocessor::RandomDownsampleFilterComponent",
        name = "random_downsample_filter",
        remappings = [
            ("input", "filtered_points/voxel_grid"),
            ("output", "filtered_points/random_downsample"),
        ],
        parameters = [
            {
                "input_frame": "base_link",
                "output_frame": "base_link",
                "sample_num": 3000,
            }
        ],
    )

    distortion_corrector_component = ComposableNode(
        package = pkg,
        plugin = "pointcloud_preprocessor::DistortionCorrectorComponent",
        name = "distortion_corrector",
        remappings = [
            ("~/input/pointcloud", "filtered_points/random_downsample"),
            ("~/input/twist", "gyro_twist_with_covariance"),
            ("~/input/imu", "imu/data_raw"),
            ("~/output/pointcloud", "filtered_points/distortion_correct"),
        ],
        parameters = [
            {
                "input_frame": "base_link",
                "output_frame": "base_link",
            }
        ],
    )

    container = ComposableNodeContainer(
        name = "pointcloud_preprocessor_container",
        namespace = ns,
        package = "rclcpp_components",
        executable = "component_container",
        composable_node_descriptions = [
            crop_box_component,
            voxel_grid_component,
            random_downsample_component,
            distortion_corrector_component,
        ],
        output = "screen",
    )

    ld = LaunchDescription()

    ld.add_action(launch_setup_func)
    ld.add_action(container)

    return ld