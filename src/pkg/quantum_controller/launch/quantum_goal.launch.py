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
            executable='lidar_listener',        
            name='lidar_listener',
            output='screen',
        ),

        # Nodo generatore dell'obiettivo (usa due LUT per X e Y)
        Node(
            package='quantum_controller',
            executable='quantum_goal_generator',  
            name='quantum_goal_generator',
            output='screen',
            parameters=[{
                'config_dir': config_dir
            }]
        ),

        # (Opzionale) Local planner in Python che riceve il goal e pubblica su /cmd_vel
        Node(
            package='nav2_simple_local_planner_py',
            executable='simple_local_planner',
            name='nav2_simple_local_planner_py',
            output='screen',
            parameters=[{
                'odom_topic': '/odom/wheels',
                'plan_topic': '/plan',
                'cmd_vel_topic': '/cmd_vel'
            }]
        ),
    ])
