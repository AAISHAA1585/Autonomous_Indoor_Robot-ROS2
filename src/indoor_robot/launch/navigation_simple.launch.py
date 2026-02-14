import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_indoor_robot = get_package_share_directory('indoor_robot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    map_file = os.path.join(pkg_indoor_robot, 'maps', 'warehouse_map.yaml')
    params_file = os.path.join(pkg_indoor_robot, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_indoor_robot, 'rviz', 'nav2.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Use Nav2 bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'true',
                'params_file': params_file,
                'autostart': 'true'
            }.items()
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])
