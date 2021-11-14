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



    # harvest_studio_robot

    dof = LaunchConfiguration('dof', default='5')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='xarm_control/FakeXArmHW')

    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')

    moveit_config_package_name = 'xarm_moveit_config'
    xarm_type = 'xarm{}'.format(dof.perform(context))

    # robot_description_parameters
    # xarm_moveit_config/launch/lib/xarm_moveit_config_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory(moveit_config_package_name), 'launch', 'lib', 'xarm_moveit_config_lib.py'))
    get_xarm_robot_description_parameters = getattr(mod, 'get_xarm_robot_description_parameters')
    robot_description_parameters = get_xarm_robot_description_parameters(
        xacro_urdf_file=PathJoinSubstitution([FindPackageShare('harvest_studio_description'), 'harvest_studio.urdf.xacro']),
        xacro_srdf_file=PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'srdf', 'xarm.srdf.xacro']),
        urdf_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns.perform(context).strip('/'),
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': dof,
            'ros2_control_plugin': ros2_control_plugin,
            'add_other_geometry': add_other_geometry,
            'geometry_type': geometry_type,
            'geometry_mass': geometry_mass,
            'geometry_height': geometry_height,
            'geometry_radius': geometry_radius,
            'geometry_length': geometry_length,
            'geometry_width': geometry_width,
            'geometry_mesh_filename': geometry_mesh_filename,
            'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
            'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
            'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
            'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
        },
        srdf_arguments={
            'prefix': prefix,
            'dof': dof,
            'add_gripper': add_gripper,
            'add_other_geometry': add_other_geometry,
        },
        arguments={
            'context': context,
            'xarm_type': xarm_type,
        }
    )

    get_xarm_state_node = Node(
        package='cpp_harvest_studio',
        executable='get_xarm_state',
        output='screen',
        name='get_xarm_state',
        parameters=[
            robot_description_parameters
        ],
    )



    nodes = [
        get_xarm_state_node,
    ]

    return nodes

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
