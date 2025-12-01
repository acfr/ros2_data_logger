import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    """
    This function is called at runtime to build the ros2 bag record command
    with the actual launch configuration values.
    """
    # Get launch configuration values
    name = LaunchConfiguration("name").perform(context)
    storage_uri = LaunchConfiguration("storage_uri").perform(context)

    # Generate timestamp for rosbag name
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    rosbag_name = name + timestamp
    rosbag_dir = os.path.join(storage_uri, rosbag_name)

    return [
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
                f"{rosbag_dir}",
                "-b",
                "5368709120",
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="name",
                default_value="rosbag_",
                description="Name of the robot or project",
            ),
            DeclareLaunchArgument(
                name="storage_uri",
                default_value=os.path.expanduser("~/"),
                description=("Path to the directory where ros2 bags will be stored"),
            ),
            # Use OpaqueFunction to set up the command at runtime
            OpaqueFunction(function=launch_setup),
        ],
    )
