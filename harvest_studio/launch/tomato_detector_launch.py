import os
import math
import json

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node, node
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import load_python_launch_file_as_module
from launch_ros.substitutions import FindPackageShare

import xacro

def launch_setup(context, *args, **kwargs):

    rs_tomato_detector_node = Node(
        package='py_harvest_studio',
        executable='tomato_detector',
        parameters=[
            {'camera_mode': 'rs'}
        ],
    )

    azure_tomato_detector_node = Node(
        package='py_harvest_studio',
        executable='tomato_detector',
        parameters=[
            {'camera_mode': 'azure'}
        ],
    )

    nodes = [
        rs_tomato_detector_node,
        azure_tomato_detector_node,
    ]

    return nodes

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])