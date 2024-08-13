# SPDX-FileCopyrightText: 2024 nacky823 youjiyongmu4@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def launch_setup(context, *args, **kwargs):

    set_use_sim_time = SetParameter(name="use_sim_time", value="true")

    return [set_use_sim_time]


def generate_launch_description():

    pkg = "ndt_scan_matcher"

    launch_setup_func = OpaqueFunction(function=launch_setup)

    params_file = os.path.join(
        get_package_share_directory(pkg),
        "config",
        "sim_iscas.param.yaml",
    )

    ndt_node = Node(
        package = pkg,
        executable = "ndt_scan_matcher",
        name = "ndt_scan_matching",
        output = "screen",
        remappings = [
            ("points_raw", "filtered_points/distortion_correct"),
            ("ekf_pose_with_covariance", "ekf_pose_with_covariance"),
            ("pointcloud_map", "map/pointcloud_map"),
            ("regularization_pose_with_covariance", "sensing/gnss/pose_with_covariance"),
            ("trigger_node_srv", "trigger_node"),
            ("pcd_loader_service", "map/get_differential_pointcloud_map"),
            ("ekf_odom", "odom"),
            ("ndt_pose", "ndt_pose"),
            ("ndt_pose_with_covariance", "ndt_pose_with_covariance"),
        ],
        parameters = [params_file],
    )

    ld = LaunchDescription()

    ld.add_action(launch_setup_func)
    ld.add_action(ndt_node)

    return ld