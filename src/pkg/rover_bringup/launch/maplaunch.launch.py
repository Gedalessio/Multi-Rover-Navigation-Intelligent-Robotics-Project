from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Ottieni path al pacchetto
    pkg_my_maps = get_package_share_directory('rover_bringup')

    # Costruisci path assoluto al file YAML della mappa
    map_file = os.path.join(pkg_my_maps, 'maps', 'my_map.yaml')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            namespace='slave',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file}]
        )
    ])