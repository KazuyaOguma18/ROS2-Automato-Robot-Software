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
    rs_ns_launch_arg = DeclareLaunchArgument(
        "rs_ns", default_value=TextSubstitution(text="rs")
    )

    rs_launch_include = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('rs_ns')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('realsense2_camera'),
                        'launch/rs_launch.py'
                    )
                )
            ),            
        ]
    )

    xarm_launch_include = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('xarm_moveit_config'),
                        'launch/xarm5_moveit_fake.launch.py'
                    )
                )
            ),
        ]
    )


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

    generate_motion_point_node = Node(
        package='cpp_harvest_studio',
        executable='generate_motion_point',
        output='screen',
        name='generate_motion_point',
        parameters=[
            robot_description_parameters
        ],
    )


    add_rviz_marker_node = Node(
        package='cpp_harvest_studio',
        executable='add_rviz_marker',
        name='harvest_studio_add_rviz_marker',
    )

    camera2dynamixel_node = Node(
        package='cpp_harvest_studio',
        executable='camera2dynamixel',
    )

    change_joint_states_node = Node(
        package='cpp_harvest_studio',
        executable='change_joint_states',
    )

    tomato_detector_node = Node(
        package='py_harvest_studio',
        executable='tomato_detector',
    )

    #q = quaternion_from_euler(0, -0.785398, -0.436332)
    #rs_camera_tf_node = Node(
    #    package='tf2_ros',
    #    executable='static_transform_publisher',
    #    arguments=['0.05084','0.16291', '0.03286', str(q[1]), str(q[2]), str(q[3]), str(q[0]),  'stand_base', 'camera_link', '100'],
    #    name='rs_camera_static_transform_publisher',
    #)



    fruit_data_processor_node = Node(
        package='py_harvest_studio',
        executable='fruit_data_processor',
    )

    hand_ros2serial_node = Node(
        package='py_harvest_studio',
        executable='hand_ros2serial',
        parameters=[
            {'hand_control_mode': 'demo'}
        ],
    )

    sample_detector_node = Node(
        package='py_harvest_studio',
        executable='sample_tomato_data_publisher',
    )

    nodes = [
        xarm_launch_include,
        #rs_ns_launch_arg,
        #rs_launch_include,
        add_rviz_marker_node,
        # camera2dynamixel_node,
        change_joint_states_node,
        #tomato_detector_node,
        # rs_camera_tf_node,
        generate_motion_point_node,
        fruit_data_processor_node,
        hand_ros2serial_node,
        # sample_detector_node,
    ]

    return nodes

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
