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
    
    ekf_config_path = os.path.join(rover_bringup_dir, 'config', 'ekf_simple.yaml')

    ekf_simple_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_simple_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        ekf_simple_node
    ])