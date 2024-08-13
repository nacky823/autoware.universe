# SPDX-FileCopyrightText: 2024 nacky823 youjiyongmu4@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import SetParameter
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):

    set_use_sim_time = SetParameter(name="use_sim_time", value=True)

    return [set_use_sim_time]


def generate_launch_description():

    pkg = "map_loader"

    map_path = "/home/nacky/mugimaru_share/data/map/iscas_sim/success/iscas_museum_3d_1.pcd"
    map_path_param = [
        {
            "pcd_paths_or_directory": [map_path],
            "pcd_metadata_path": map_path,
        }
    ]

    launch_setup_func = OpaqueFunction(function=launch_setup)

    params_file = os.path.join(
        get_package_share_directory(pkg),
        "config",
        "sim_iscas.param.yaml",
    )

    pcd_map_loader_component = ComposableNode(
        package = pkg,
        plugin = "PointCloudMapLoaderNode",
        name = "pcd_map_loader",
        remappings = [
            ("output/pointcloud_map", "map/pointcloud_map"),
            ("service/get_partial_pcd_map", "map/get_partial_pointcloud_map"),
            ("service/get_selected_pcd_map", "map/get_selected_pointcloud_map"),
        ],
        parameters = map_path_param + [params_file],
    )

    container = ComposableNodeContainer(
        name = "map_loader_container",
        namespace = "",
        package = "rclcpp_components",
        executable = "component_container",
        composable_node_descriptions = [
            pcd_map_loader_component,
        ],
        output = "screen",
    )

    ld = LaunchDescription()

    ld.add_action(launch_setup_func)
    ld.add_action(container)

    return ld