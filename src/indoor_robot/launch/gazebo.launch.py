import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_indoor_robot = get_package_share_directory('indoor_robot')

    # Paths to our files
    world_file = os.path.join(pkg_indoor_robot, 'worlds', 'warehouse.world')
    urdf_file = os.path.join(pkg_indoor_robot, 'models', 'robot.urdf.xacro')

    # Process the URDF file with xacro
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Spawn robot in Gazebo (with delay to ensure Gazebo is ready)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'warehouse_robot',
            '-topic', 'robot_description',
            '-x', '-3.0',
            '-y', '0.0',
            '-z', '0.15'
        ],
        output='screen'
    )

    # Delay spawning to ensure everything is ready
    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_robot]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        delayed_spawn
    ])
