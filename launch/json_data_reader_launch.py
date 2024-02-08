# Description: Launch file for data_reader node with the correct json file
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rov_rviz'),
        'config', # repertoire
        'equilateral_triangle.yaml')

    config_rviz = os.path.join(
        get_package_share_directory('rov_rviz'),
        'config', # repertoire
        'rviz_config.rviz'
    )

    node = Node(
        package='rov_rviz',
        namespace ='',
        executable = 'json_data_reader',
        name = 'json_data_reader',
        parameters = [config]
    )

    node_rviz = Node(
        package='rviz2',
        namespace ='',
        executable ='rviz2',
        name='Rov_rviz2',
        arguments=['-d'+config_rviz]
    )

    return LaunchDescription([node,node_rviz])