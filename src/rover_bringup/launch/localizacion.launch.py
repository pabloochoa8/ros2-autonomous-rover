import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rover_bringup_dir = get_package_share_directory('rover_bringup')
    ekf_config_path = os.path.join(rover_bringup_dir, 'config', 'ekf.yaml')

    # EKF Local (Odometría e IMU -> odom a base_link)
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=[
            ekf_config_path,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    # EKF Global (Odometría, IMU y ArUcos -> map a odom)
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[
            ekf_config_path,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        ekf_local_node,
        ekf_global_node
    ])