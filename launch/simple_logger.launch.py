# Copyright 2024 -
# Australian Centre for Robotic
#
# Authored by Jerome Justin
#

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
import datetime


def generate_launch_description():
    name_arg = LaunchConfiguration("name")

    if not name_arg or not name_arg.perform({}):
        name = "rosbag_"
    else:
        name = name_arg.perform({})

    storage_uri_arg = LaunchConfiguration("storage_uri")

    if not storage_uri_arg or not storage_uri_arg.perform({}):
        storage_uri = os.path.expanduser("~/")
    else:
        storage_uri = storage_uri_arg.perform({})

    # Generate a timestamp for the rosbag name
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    rosbag_name = name + timestamp

    rosbag_dir = os.path.join(storage_uri, rosbag_name)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="name",
                default_value=rosbag_name,
                description="Name of the robot or project",
            ),
            DeclareLaunchArgument(
                name="storage_uri",
                default_value=rosbag_dir,
                description="Path to the directory where ros2 bags will be stored",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-s",
                    "mcap",
                    "--storage-preset-profile",
                    "zstd_fast",
                    "-a",
                    "-o",
                    f"{rosbag_dir}/{rosbag_name}",
                    "-b",
                    "5368709120",
                ],
                output="screen",
            ),
        ],
    )
