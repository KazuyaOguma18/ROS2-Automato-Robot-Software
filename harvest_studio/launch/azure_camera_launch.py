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

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


def launch_setup(context, *args, **kwargs):
    azure_ns_launch_arg = DeclareLaunchArgument(
        "azure_ns", default_value=TextSubstitution(text="azure")
    )
    
    azure_launch_include = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('azure_ns')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('azure_kinect_ros_driver'),
                        'launch/driver.launch.py'
                    )
                )
            ),            
        ]
    )
    
    q = quaternion_from_euler(1.57, 0, 0)
    azure_camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.04045', '0.072', '1.57', '0', '0', 'azure_camera_link', 'camera_base'],
        name='azure_camera_static_transform_publisher',
    )
    
    nodes = [
        azure_ns_launch_arg,
        azure_launch_include,
        azure_camera_tf_node,
    ]

    return nodes

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])    