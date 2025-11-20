import datetime
import os

import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    NotEqualsSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

"""
Used to load parameters for composable nodes from a standard param file
"""


def dump_params(param_file_path, node_name):
    with open(param_file_path, "r") as file:
        return [yaml.safe_load(file)[node_name]["ros__parameters"]]


def launch_setup(context, *args, **kwargs):
    """
    This function is called at runtime to set up the composable nodes
    with the actual launch configuration values.
    """
    # Get launch configuration values
    name = LaunchConfiguration("name").perform(context)
    storage_uri = LaunchConfiguration("storage_uri").perform(context)
    params_file_uri = LaunchConfiguration("params_file_uri").perform(context)

    # Generate timestamp for rosbag name
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    rosbag_name = name + timestamp
    container_rosbag_dir = os.path.join(storage_uri, rosbag_name)

    # Load parameters and override the storage URI
    recorder_params = dump_params(params_file_uri, "recorder")
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

    return [
        ComposableNodeContainer(
            condition=IfCondition(
                EqualsSubstitution(LaunchConfiguration("container"), "")
            ),
            package="rclcpp_components",
            executable="component_container",
            name="logging_container",
            namespace="",
            composable_node_descriptions=composable_nodes,
        ),
        LoadComposableNodes(
            condition=IfCondition(
                NotEqualsSubstitution(LaunchConfiguration("container"), "")
            ),
            composable_node_descriptions=composable_nodes,
            target_container=LaunchConfiguration("container"),
        ),
        # If a container name is not provided,
        # set the name of the container launched above for logging node
        SetLaunchConfiguration(
            condition=IfCondition(
                EqualsSubstitution(LaunchConfiguration("container"), "")
            ),
            name="container",
            value="logging_container",
        ),
    ]


def generate_launch_description():
    # Get default parameter file path
    params_file = os.path.join(
        get_package_share_directory("ros2_data_logger"),
        "config",
        "recorder_params.yaml",
    )

    return launch.LaunchDescription(
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
            DeclareLaunchArgument(
                name="params_file_uri",
                default_value=params_file,
                description=("Path to the parameter file for the logger nodes"),
            ),
            DeclareLaunchArgument(
                name="container",
                default_value="",
                description=(
                    "Name of an existing node container to load "
                    "launched nodes into. If unset, a new container "
                    "will be created."
                ),
            ),
            # Use OpaqueFunction to set up nodes at runtime
            OpaqueFunction(function=launch_setup),
        ]
    )
