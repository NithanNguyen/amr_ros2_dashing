import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ==========================================
    # 1. KHAI BÁO BIẾN VÀ ĐƯỜNG DẪN
    # ==========================================
    
    pkg_elder_robot = get_package_share_directory('elder_robot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Đường dẫn files
    world_file = os.path.join(pkg_elder_robot, 'worlds', 'room2.world')
    urdf_file = os.path.join(pkg_elder_robot, 'urdf', 'sim.urdf')
    nav2_params_file = os.path.join(pkg_elder_robot, 'config', 'nav2_config.yaml')
    ekf_params_file = os.path.join(pkg_elder_robot, 'config', 'ekf.yaml')
    rviz_config_file = os.path.join(pkg_elder_robot, 'config', 'slam_config.rviz')
    
    default_map_path = os.path.join(pkg_elder_robot, 'maps', 'my_map.yaml')
    map_file = LaunchConfiguration('map', default=default_map_path)

    # Cấu hình Gazebo model path
    model_path = os.path.join(pkg_elder_robot, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += os.pathsep + model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = model_path

    # ==========================================
    # 2. ĐỊNH NGHĨA CÁC NODE
    # ==========================================

    # Node 1: Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0
        }],
        arguments=[urdf_file]
    )

    # Node 2: Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Node 3: Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        node_executable='spawn_entity.py',
        arguments=[
            '-entity', 'elderbot',
            '-file', urdf_file,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Node 4: IMU Reader (Custom node để đọc và hiển thị IMU data)
    imu_reader = Node(
        package='elder_robot',
        node_executable='imu_reader.py',
        name='imu_reader',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Node 5: Robot Localization - EKF (Sensor Fusion: Odom + IMU)
    ekf_localization = Node(
        package='robot_localization',
        node_executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/odometry/filtered', '/odom/filtered')
        ]
    )

    # Node 6: Nav2 Bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'nav2_bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )

    # Node 7: RViz2
    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file]
    )

    # ==========================================
    # 3. QUẢN LÝ THỜI GIAN KHỞI CHẠY
    # ==========================================
    
    # Chờ 5s để Gazebo load xong -> Spawn Robot
    delayed_spawn = TimerAction(period=7.0, actions=[spawn_entity])
    
    # Chờ 7s -> Chạy IMU Reader và EKF
    delayed_imu_ekf = TimerAction(
        period=10.0, 
        actions=[imu_reader, ekf_localization]
    )
    
    # Chờ 8s -> Chạy Navigation
    delayed_nav2 = TimerAction(period=15.0, actions=[nav2_bringup])

    # ==========================================
    # 4. RETURN LAUNCH DESCRIPTION
    # ==========================================
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'map',
            default_value=default_map_path,
            description='Full path to map file to load'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Launch các nodes
        gazebo,
        robot_state_publisher,
        delayed_spawn,
        delayed_imu_ekf,
        delayed_nav2,
        rviz2
    ])