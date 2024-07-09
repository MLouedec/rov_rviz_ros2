# Description: Launch file for bag2csv node
import os
import launch

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rosbag_folder = "/home/morgan/Documents/Code_these/rosbags/"
    # rosbag_file = "/guerledan_fevrier_2024/vx_const1/vx_const1_0"
    # rosbag_file = "/guerledan_fevrier_2024/vx_const2/vx_const2_0"
    # rosbag_file = "/guerledan_fevrier_2024/vy_const1/vy_const1_0"
    # rosbag_file = "/guerledan_fevrier_2024/vy_const2/vy_const2_0"

    # rosbag_file = "/guerledan_fevrier_2024/vx_stop1/vx_stop1_0"
    # rosbag_file = "/guerledan_fevrier_2024/vx_stop2/vx_stop2_0"

    # rosbag_file = "/guerledan_fevrier_2024/vy_stop1/vy_stop1_0"
    # rosbag_file = "/guerledan_fevrier_2024/vy_stop2/vy_stop2_0"

    # rosbag_file = "/guerledan_fevrier_2024/vy_stop3/vy_stop3_0"
    rosbag_file = "/guerledan_fevrier_2024/vx_stop3/vx_stop3_0"

    # rosbag_file = "/guerledan_fevrier_2024/vx_vide/vx_vide2_0"
    # rosbag_file = "/guerledan_fevrier_2024/vy_vide/vy_vide_0"

    # rosbag_file = "/guerledan_fevrier_2024/vxconst75/vxconst75_2_0"
    # rosbag_file = "guerledan_fevrier_2024/vxconst85/vxconst85_0"
    # rosbag_file = "guerledan_fevrier_2024/vxconst110/vxconst110_0"

    # rosbag_file = "/guerledan_fevrier_2024/vyconst75/vyconst75_0"
    # rosbag_file = "/guerledan_fevrier_2024/vyconst85/vyconst85_0"
    # rosbag_file = "/guerledan_fevrier_2024/vyconst110/vyconst110_0"

    rosbag_full_path = rosbag_folder + rosbag_file+".db3"
    csv_full_path = rosbag_folder + rosbag_file+".csv"

    # config = os.path.join(
    #     get_package_share_directory('rov_rviz'),
    #     'config', # repertoire
    #     'bag2csv.yaml')

    node = Node(
        package='rov_rviz_ros2',
        namespace ='',
        executable = 'bag2csv',
        name = 'bag2csv',
        parameters = [
            {"csv_full_path": csv_full_path}
        ]
    )

    # node_rosbag = Node(
    #     package='bag',
    #     namespace ='',
    #     executable ='play',
    #     name='rosbag_player',
    #     arguments="$(arg rosbag_folder)$(arg rosbag_file) --topics /mavros/global_position.local /mavros/rc/override"
    # )
    rosbag = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'play', rosbag_full_path,'--topics', '/mavros/global_position/local', '/mavros/rc/override'],
        # cmd='ros2 bag play '+ rosbag_full_path+ ' --topics  /mavros/global_position/local /mavros/rc/override',
        output='screen'
    )

    # return LaunchDescription([node,node_rosbag])
    # return LaunchDescription([node])
    return LaunchDescription([rosbag,node])