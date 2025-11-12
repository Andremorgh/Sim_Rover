from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_simple_local_planner_py')
    params_file = os.path.join(pkg_share, 'params', 'local_planner_params.yaml')

    simple_planner = Node(
        package='nav2_simple_local_planner_py',
        executable='simple_local_planner',
        name='nav2_simple_local_planner_py',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([simple_planner])