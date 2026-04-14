import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    osr_bringup_dir = get_package_share_directory('osr_bringup')

    # Motores, servos, encoders y odometría (Roboclaw)
    osr_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(osr_bringup_dir, 'launch', 'osr_launch.py')
        ),
        launch_arguments={'enable_odometry': 'true'}.items()
    )

    # IMU BNO055 por I2C
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055_node',
        parameters=[
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

    # TF estático base_link -> base_footprint (requerido por Nav2)
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    # TF estático base_link -> camera (cámara frontal, x=0.09, z=0.2)
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
        bno055_node,
        base_footprint_tf,
        camera_tf,
    ])
