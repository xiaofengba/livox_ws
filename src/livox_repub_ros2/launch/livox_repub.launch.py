'''
Author: nuc 1078456645@qq.com
Date: 2025-11-06 14:08:28
LastEditors: nuc 1078456645@qq.com
LastEditTime: 2025-11-17 13:03:58
FilePath: /livox_ws/src/livox_repub_ros2/launch/livox_repub.launch.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    livox_repub_node = Node(
        package='livox_repub_ros2',
        executable='livox_repub_node',
        name='livox_repub_node',
        output='screen'
    )
    livox_imu = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        parameters=[{'use_mag': False, 
                        'world_frame':'enu', 
                        'publish_tf':False,
                        'yaw_offset': 0.0,
                        'fixed_frame': 'odom',
                        'orientation_stddev': 0.1
                        }],
        remappings=[('imu/data_raw', '/livox/imu/processed')])

    
    return LaunchDescription([
        livox_repub_node, livox_imu
    ])
