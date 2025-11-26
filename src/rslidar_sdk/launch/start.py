'''
Author: nuc 1078456645@qq.com
Date: 2025-11-11 13:07:17
LastEditors: nuc 1078456645@qq.com
LastEditTime: 2025-11-11 14:28:31
FilePath: /livox_ws/src/rslidar_sdk/launch/start.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rviz_config=get_package_share_directory('rslidar_sdk')+'/rviz/rviz2.rviz'

    config_file = '/home/nuc/data/code/livox_ws/src/rslidar_sdk/config/config.yaml' # your config file path
    
    return LaunchDescription([
        Node(namespace='rslidar_sdk', package='rslidar_sdk', executable='rslidar_sdk_node', output='screen', parameters=[{'config_path': config_file}]),
        Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config])
    ])
