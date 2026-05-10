import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rover_bringup_dir = get_package_share_directory('rover_bringup')
    osr_bringup_dir = get_package_share_directory('osr_bringup')

    # 1. Rover Físico (Odometría + Motores)
    osr_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(osr_bringup_dir, 'launch', 'osr_launch.py') 
        ),
        launch_arguments={'enable_odometry': 'true'}.items()
    )
    
    # 2. Cámara (V4L2)
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[
            {'video_device': '/dev/video0'}, 
            {'image_size': [640, 480]},      
            {'framerate': 10}                
        ],
        output='screen'
    )

    # 3. Detección ArUcos
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
    
    # 4. Mock ArUco a EKF
    mock_aruco_node = ExecuteProcess(
        cmd=['python3', os.path.join(os.getcwd(), 'src', 'rover_bringup', 'config', 'mock_aruco.py'), '--ros-args', '-p', 'use_sim_time:=false'],
        output='screen'
    )

    # 5. Localización Simplificada (El nuevo archivo)
    localizacion_simple_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_bringup_dir, 'launch', 'localizacion_simple.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 6. Controlador Chapuza (Pure Pursuit Básico)
    pure_pursuit_node = ExecuteProcess(
        cmd=['python3', os.path.join(os.getcwd(), 'src', 'rover_bringup', 'src', 'simple_pure_pursuit.py')],
        output='screen'
    )

    # 7. TF de Cámara (Igual que antes)
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=['0.09', '0.0', '0.2', '0', '0', '0', 'base_link', 'camera']
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=False),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        osr_control_launch,
        v4l2_camera_node,
        aruco_node,
        mock_aruco_node,
        localizacion_simple_launch,
        pure_pursuit_node,
        camera_tf
    ])