import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('pc_merger')
    config_file = os.path.join(get_package_share_directory('pc_merger'), 'config', 'config.yaml')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')
    use_4_lidar = LaunchConfiguration('use_4_lidar', default=True)
    
    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        Node(
            package='pc_merger',
            executable='pc_merger',
            parameters=[config_file, {'use_4_lidar': use_4_lidar}],
            output='screen'
        )
    ])