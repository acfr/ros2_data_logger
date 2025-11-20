import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch.conditions import LaunchConfigurationNotEquals, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration

import yaml
import os
import datetime

"""
Used to load parameters for composable nodes from a standard param file
"""


def dump_params(param_file_path, node_name):
    with open(param_file_path, "r") as file:
        return [yaml.safe_load(file)[node_name]["ros__parameters"]]


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

    container_rosbag_dir = os.path.join(storage_uri, rosbag_name)

    # If params_file argument is empty or the provided path does not exist use default
    params_file_arg = LaunchConfiguration("params_file_uri")
    if not params_file_arg or not os.path.exists(params_file_arg.perform({})):
        # Define the path to the parameter file
        params_file = os.path.join(
            get_package_share_directory("ros2_data_logger"),
            "config",
            "recorder_params.yaml",
        )
    else:
        params_file = params_file_arg.perform({})

    # Load parameters and override the storage URI
    recorder_params = dump_params(params_file, "recorder")
    recorder_params[0]["storage"]["uri"] = container_rosbag_dir

    composable_logger = ComposableNode(
        package="rosbag2_transport",
        plugin="rosbag2_transport::Recorder",
        namespace="logger",
        name="recorder",
        parameters=recorder_params,
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # You can add more composable nodes here if needed
    composable_nodes = [composable_logger]

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                name="name",
                default_value=rosbag_name,
                description="Name of the robot or project",
            ),
            DeclareLaunchArgument(
                name="storage_uri",
                default_value=container_rosbag_dir,
                description="Path to the directory where ros2 bags will be stored",
            ),
            DeclareLaunchArgument(
                name="params_file_uri",
                default_value=params_file,
                description="Path to the parameter file for the logger nodes",
            ),
            DeclareLaunchArgument(
                name="container",
                default_value="",
                description=(
                    "Name of an existing node container to load launched nodes into. "
                    "If unset, a new container will be created."
                ),
            ),
            ComposableNodeContainer(
                condition=LaunchConfigurationEquals("container", ""),
                package="rclcpp_components",
                executable="component_container",
                name="logging_container",
                namespace="",
                composable_node_descriptions=composable_nodes,
            ),
            LoadComposableNodes(
                condition=LaunchConfigurationNotEquals("container", ""),
                composable_node_descriptions=composable_nodes,
                target_container=LaunchConfiguration("container"),
            ),
            # If a container name is not provided,
            # set the name of the container launched above for logging node
            SetLaunchConfiguration(
                condition=LaunchConfigurationEquals("container", ""),
                name="container",
                value="logging_container",
            ),
        ]
    )
