import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_indoor_robot = get_package_share_directory('indoor_robot')
    
    slam_params_file = os.path.join(
        pkg_indoor_robot,
        'config',
        'slam_params.yaml'
    )
    
    rviz_config_file = os.path.join(
        pkg_indoor_robot,
        'rviz',
        'slam.rviz'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time}
            ]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
