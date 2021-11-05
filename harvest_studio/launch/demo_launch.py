import os
import math

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

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

def generate_launch_description():
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

    generate_motion_point_node = Node(
        package='cpp_harvest_studio',
        executable='generate_motion_point',
    )

    fruit_data_processor_node = Node(
        package='py_harvest_studio',
        executable='fruit_data_processor',
    )

    hand_ros2serial_node = Node(
        package='py_harvest_studio',
        executable='hand_ros2serial',
    )

    sample_detector_node = Node(
        package='py_harvest_studio',
        executable='sample_tomato_data_publisher',
    )



    return LaunchDescription([
        #rs_ns_launch_arg,
        #rs_launch_include,
        add_rviz_marker_node,
        # camera2dynamixel_node,
        change_joint_states_node,
        #tomato_detector_node,
        # rs_camera_tf_node,
        # generate_motion_point_node,
        # fruit_data_processor_node,
        # hand_ros2serial_node,
        sample_detector_node,
    ])