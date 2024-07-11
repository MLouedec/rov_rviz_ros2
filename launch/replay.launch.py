# Description: Launch file for replay node
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
    # rosbag_file = "/guerledan_fevrier_2024/vx_stop3/vx_stop3_0"

    # rosbag_file = "/guerledan_fevrier_2024/vx_vide/vx_vide2_0"
    # rosbag_file = "/guerledan_fevrier_2024/vy_vide/vy_vide_0"

    # rosbag_file = "/guerledan_fevrier_2024/vxconst75/vxconst75_2_0"
    # rosbag_file = "guerledan_fevrier_2024/vxconst85/vxconst85_0"
    # rosbag_file = "guerledan_fevrier_2024/vxconst110/vxconst110_0"

    # rosbag_file = "/guerledan_fevrier_2024/vyconst75/vyconst75_0"
    # rosbag_file = "/guerledan_fevrier_2024/vyconst85/vyconst85_0"
    # rosbag_file = "/guerledan_fevrier_2024/vyconst110/vyconst110_0"

    # rosbag_file = "/rosbag-submeeting-june-2024/tests_mer_usbl_lion_de_mer/formation_sous_eau_1/rosbag2_2024_05_29-11_23_00/rosbag2_2024_05_29-11_23_00_0.db3"
    # rosbag_file = "/rosbag-submeeting-june-2024/tests_mer_usbl_lion_de_mer/formation_sous_eau_2/rosbag2_2024_05_29-11_27_47/rosbag2_2024_05_29-11_27_47_0.db3"
    rosbag_file = "/rosbag-submeeting-june-2024/formation_rovs_port_st_raphael/2_rovs_formation_st_raphael_pwm_80_n3/rosbag2_2024_05_28-16_21_29/rosbag2_2024_05_28-16_21_29_0.db3"



    rosbag_full_path = rosbag_folder + rosbag_file
    node = Node(
        package='rov_rviz_ros2',
        namespace ='',
        executable = 'replay',
        name = 'rov_rviz_replay',
        parameters = [
            {"rovA_name": "inky"},
            {"rovB_name": "blinky"}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/morgan/Documents/Code_these/ros2/src/rov_rviz_ros2/config/replay.rviz']
    )

    rosbag = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'play', rosbag_full_path,'--topics', '/bouee/usbl', '/bouee/imu_data',
            "/inky/mavros/global_position/local", "/blinky/mavros/global_position/local",
             "/inky/coord_objectif","/blinky/coord_objectif",
             "/inky/mavros/rc/override","/blinky/mavros/rc/override"],
        output='screen'
    )

    return LaunchDescription([rosbag,node,rviz_node])