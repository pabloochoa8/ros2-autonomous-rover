import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rover_bringup_dir = get_package_share_directory('rover_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')

    # Cámara USB V4L2
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[
            {'video_device': '/dev/video0'},
            {'image_size': [640, 480]}
        ],
        output='screen'
    )

    # LiDAR YDLidar
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_dir, 'launch', 'ydlidar_launch.py')
        )
    )

    # Localización: EKF local y global
    localizacion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_bringup_dir, 'launch', 'localizacion.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Detección ArUco
    aruco_params_path = os.path.join(rover_bringup_dir, 'config', 'aruco.yaml')
    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node',
        parameters=[aruco_params_path, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/camera/color/image_raw', '/image_raw'),
            ('/camera/color/camera_info', '/camera_info')
        ],
        output='screen'
    )

    # Puente ArUco -> EKF (posiciones mapa 7x7)
    mock_aruco_node = ExecuteProcess(
        cmd=['python3', os.path.join(os.getcwd(), 'src', 'rover_bringup', 'config', 'mock_aruco.py'), '--ros-args', '-p', 'use_sim_time:=false'],
        output='screen'
    )

    # Nav2 (AMCL, planificador, costmaps)
    map_file = os.path.join(rover_bringup_dir, 'maps', 'map.yaml')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(rover_bringup_dir, 'config', 'nav2_params.yaml')
        }.items()
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=False),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        v4l2_camera_node,
        lidar_launch,
        localizacion_launch,
        aruco_node,
        mock_aruco_node,
        nav2_launch,
    ])
