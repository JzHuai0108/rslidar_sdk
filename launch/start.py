from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rviz_config=get_package_share_directory('rslidar_sdk')+'/rviz/rviz2.rviz'
    use_rviz = LaunchConfiguration('use_rviz')

    config_file = '' # your config file path
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz with the packaged LiDAR configuration.',
        ),
        Node(namespace='rslidar_sdk', package='rslidar_sdk', executable='rslidar_sdk_node', output='screen', parameters=[{'config_path': config_file}]),
        Node(
            namespace='rviz2',
            package='rviz2',
            executable='rviz2',
            arguments=['-d',rviz_config],
            condition=IfCondition(use_rviz),
        )
    ])
