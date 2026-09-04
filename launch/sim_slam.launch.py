import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('elder_robot')
    
    pkg_share = get_package_share_directory('elder_robot')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    slam_config_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'sim.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'room2.world')
    rviz_config_file = os.path.join(pkg_share, 'config', 'slam_config.rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file,
        }.items(),
    )
    
    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': True}],
        arguments=[urdf_file],
        output='screen',
    )

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        node_executable='spawn_entity.py',
        arguments=[
            '-entity', 'elderbot',
            '-file', urdf_file,
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        node_executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_file,
            {'use_sim_time': True}   
        ]
    )

    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_file]
    )
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_entity])
    delayed_slam = TimerAction(period=10.0, actions=[slam_toolbox])

    return LaunchDescription([
        gazebo,
        robot_state_pub,
        delayed_spawn,
        delayed_slam,
        rviz2
    ])
