import os

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

def generate_launch_description():
    rs_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch/rs_launch.py'
            )
        )
    )

    add_rviz_marker_node = Node(
        package='cpp_harvest_studio',
        executable='add_rviz_marker',
        name='harvest_studio_add_rviz_marker'
    )

    camera2dynamixel_node = Node(
        package='cpp_harvest_studio',
        executable='camera2dynamixel'
    )

    change_joint_states_node = Node(
        package='cpp_harvest_studio',
        executable='change_joint_states'
    )

    tomato_detector_node = Node(
        package='py_harvest_studio',
        executable='tomato_detector'
    )

    generate_motion_point_node = Node(
        package='cpp_harvest_studio',
        executable='generate_motion_point'
    )

    fruit_data_processor_node = Node(
        package='py_harvest_studio',
        executable='fruit_data_processor'
    )

    hand_ros2serial_node = Node(
        package='py_harvest_studio',
        executable='hand_ros2serial'
    )



    return LaunchDescription([
        rs_launch_include,
        add_rviz_marker_node,
        camera2dynamixel_node,
        change_joint_states_node,
        tomato_detector_node,
        # generate_motion_point_node,
        # fruit_data_processor_node,
        # hand_ros2serial_node,
    ])