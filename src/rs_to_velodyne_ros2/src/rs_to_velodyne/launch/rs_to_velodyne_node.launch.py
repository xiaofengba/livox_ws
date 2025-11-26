from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    input_topic_arg = DeclareLaunchArgument('input_topic', default_value='/rslidar_points')
    input_type_arg = DeclareLaunchArgument('input_type', default_value='XYZIRT') # XYZI XYZIRT
    output_frame_arg = DeclareLaunchArgument('output_frame', default_value='rslidar')
    output_topic_arg = DeclareLaunchArgument('output_topic', default_value='/rslidar_points_repub')
    output_type_arg = DeclareLaunchArgument('output_type', default_value='XYZIRT') # XYZI XYZIR XYZIRT
    

    convert_node = Node(
        package='rs_to_velodyne', 
        executable='rs_to_velodyne_node', 
        output='screen',
        parameters=[
            {'input_topic': LaunchConfiguration('input_topic')},
            {'input_type': LaunchConfiguration('input_type')},
            {'output_frame': LaunchConfiguration('output_frame')},
            {'output_topic': LaunchConfiguration('output_topic')},
            {'output_type': LaunchConfiguration('output_type')}
        ]
    )

    return LaunchDescription([
        input_topic_arg,
        input_type_arg,
        output_frame_arg,
        output_topic_arg,
        output_type_arg,
        convert_node
    ])
