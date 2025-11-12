from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('quantum_controller')
    config_dir = os.path.join(pkg_share, 'config')

    return LaunchDescription([
        # Nodo LIDAR listener
        Node(
            package='quantum_controller',
            executable='lidar_listener_sim',        # deve corrispondere a entry_point in setup.py
            name='lidar_listener',
            output='screen',
        ),

        # Nodo controllore quantistico
        Node(
            package='quantum_controller',
            executable='quantum_controller_node_sim',
            name='quantum_controller',
            output='screen',
            parameters=[{
                'config_dir': config_dir
            }]
        ),
    ])
