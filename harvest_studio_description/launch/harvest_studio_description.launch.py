import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
import xacro

def launch_setup(context, *args, **kwargs):
    share_dir_path = os.path.join(get_package_share_directory('harvest_studio_description'))
    xacro_path = os.path.join(share_dir_path, 'harvest_studio.urdf.xacro')
    urdf_path = os.path.join(share_dir_path, 'harvest_studio.urdf')

    doc = xacro.process_file(xacro_path)
    robot_desc = doc.toprettyxml(indent='   ')

    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()


    rsp = Node(package='robot_state_publisher',
               executable = 'robot_state_publisher',
               output = 'both',
               arguments = [urdf_path])

    jsp = Node(package='joint_state_publisher',
               executable='joint_state_publisher')

    rviz = Node(package= 'rviz2',
                executable= 'rviz2')
    
    return rsp, rviz



def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])