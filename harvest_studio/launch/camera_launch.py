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
        arguments=['0', '0.0618', '0.0405', '1.57', '0', '0', 'azure_camera_link', 'camera_base'],
        name='azure_camera_static_transform_publisher',
    )

    rs_camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.16291', '-0.25084', '0.03286', '0', '-0.436332', '-0.785398', 'stand_base', 'camera_link'],
        name='rs_camera_static_transform_publisher',
    )

    rs_ns_launch_arg = DeclareLaunchArgument(
        "rs_ns", default_value=TextSubstitution(text="rs")
    )

    # realsenseのモデルの出力（azure kinectとの衝突により動作せず)
    rs_model_launch_arg = DeclareLaunchArgument(
        "model", default_value=TextSubstitution(text="d415")
    )

    rs_robot_desctiption_include = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('rs_ns')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(   
                    os.path.join(
                        get_package_share_directory('realsense2_description'),
                        'launch/view_model.launch.py'
                    )
                )
            ),
        ]
    )

    # 圧縮画像の解凍用ノード
    rs_color_unzip_node = Node(
        namespace='rs_color',
        package='image_transport',
        executable='republish',
        arguments=['compressed', 'raw'],
        remappings=[
            ("in/compressed", '/camera/color/image_raw/compressed'),
            ("out", 'image_com'),
        ]
    )


    rs_depth_unzip_node = Node(
        namespace='rs_depth',
        package='image_transport',
        executable='republish',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/camera/aligned_depth_to_color/image_raw/compressed'),
            ('out', 'image_com'),
        ]
    )

    nodes = [
        azure_ns_launch_arg,
        azure_launch_include,
        azure_camera_tf_node,
        rs_camera_tf_node,
        # rs_ns_launch_arg,
        # rs_model_launch_arg,
        # rs_robot_desctiption_include,
        rs_depth_unzip_node,
        rs_color_unzip_node,
    ]

    return nodes

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])    