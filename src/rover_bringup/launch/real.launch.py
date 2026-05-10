import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    # 1. Configuración de tiempo (FALSO para el robot real)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Directorios de paquetes
    rover_bringup_dir = get_package_share_directory('rover_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    osr_bringup_dir = get_package_share_directory('osr_bringup')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')
    # (Hemos eliminado el directorio de la astra_camera porque ya no nos hace falta)

    # ========================================================================
    # ESPACIO PARA EL HARDWARE FÍSICO
    # ========================================================================
    # 1.5 Nodo/Launch del control real del Open Source Rover (OSR)
    # Localización basada en IMU + ArUcos + LiDAR: odometría de ruedas desactivada.
    osr_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(osr_bringup_dir, 'launch', 'osr_launch.py')
        ),
        launch_arguments={'enable_odometry': 'false'}.items()
    )
    
    # 1.6 NUEVO: Nodo de la cámara universal V4L2 (Sustituye a la Astra)
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[
            {'video_device': '/dev/video0'}, # Apuntamos al dispositivo que descubriste que funciona
            {'image_size': [640, 480]},      # Resolución ideal para el Hub y los ArUcos
            {'framerate': 10}                # Bajamos de 30 a 10 FPS para reducir carga
        ],
        output='screen'
    )

    # 1.7 Nodo del YDLidar
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_dir, 'launch', 'ydlidar_launch.py') 
        )
    )

    # 1.8 NUEVO: Nodo IMU BNO055
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055_node',
        parameters=[
            # Forzamos el modo I2C
            {'connection_type': 'i2c'},
            {'i2c_bus': 1},
            {'frame_id': 'imu_link'}
        ],
        remappings=[
            ('bno055/imu', 'imu'),
            ('bno055/calib_status', 'imu/calib_status')
        ],
        output='screen'
    )
    # ========================================================================

    # 2. Localización (EKF local y global)
    localizacion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_bringup_dir, 'launch', 'localizacion.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. Reconocimiento ArUco (ros2_aruco)
    aruco_params_path = os.path.join(rover_bringup_dir, 'config', 'aruco.yaml')
    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node',
        parameters=[aruco_params_path, {'use_sim_time': use_sim_time}],
        # Le decimos al nodo de ArUco que lea la imagen del nuevo driver genérico
        remappings=[
            ('/camera/color/image_raw', '/image_raw'),
            ('/camera/color/camera_info', '/camera_info')
        ],
        output='screen'
    )
    
    # 4. Puente: Procesador ArUco -> EKF
    mock_aruco_node = ExecuteProcess(
        cmd=['python3', os.path.join(os.getcwd(), 'src', 'rover_bringup', 'config', 'mock_aruco.py'), '--ros-args', '-p', 'use_sim_time:=false'],
        output='screen'
    )
	# 5. Nav2 (Navegación Autónoma)
    map_file = os.path.join(rover_bringup_dir, 'maps', 'map.yaml')
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(rover_bringup_dir, 'config', 'nav2_params.yaml') # Descomenta esto si usas un archivo de parámetros específico
        }.items()
    )
    nav2_remapped_launch = SetParameter(name='remappings', value=[('/cmd_vel', '/cmd_vel_intuitive')])
    
    # 6. RViz para visualizar sensores y TF en tiempo real
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    # 1.10 TF estático base_link -> camera (cámara frontal, misma posición que laser_frame)
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=['0.09', '0.0', '0.2', '0', '0', '0', 'base_link', 'camera']
    )

    # 1.9 NUEVO: Puente TF entre base_footprint (Nav2) y base_link (Rover)
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'] # <--- CAMBIO AQUÍ
    )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf',
        arguments=['0', '0', '0', '1.57', '0', '0', 'base_link', 'laser_frame']
    )

    odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    odom_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='o_b_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    return LaunchDescription([
        # Forzamos el uso del reloj del sistema (hardware real)
        SetParameter(name='use_sim_time', value=False),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        osr_control_launch,
        v4l2_camera_node,      # Arrancamos la nueva cámara
        lidar_launch,
        bno055_node,           # Arrancamos la IMU
        localizacion_launch,
        aruco_node,
        mock_aruco_node,
        odom_tf,
        odom_base_tf,
        lidar_tf,
        camera_tf,
        base_footprint_tf,
        #rviz_launch
        nav2_launch
    ])