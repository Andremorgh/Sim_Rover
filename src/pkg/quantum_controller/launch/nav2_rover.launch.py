from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Trova la directory del pacchetto
    pkg_share = FindPackageShare('quantum_controller').find('quantum_controller')

    # File di configurazione
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # Parametro opzionale per il tempo simulato
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # --- Static TF Broadcasters ---
    # map -> rover/odom
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'rover/map', 'rover/odom'],
        output='screen'
    )

    # rover/odom -> rover/base_footprint
    static_tf_odom_to_basefootprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_to_basefootprint_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'rover/odom', 'rover/base_footprint'],
        output='screen'
    )

    # --- Planner Server ---
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # --- Controller Server ---
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # --- Lifecycle Manager ---
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['controller_server', 'planner_server']
        }],
    )

    # --- LaunchDescription ---
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True',
                              description='Use simulation time if true'),

        static_tf_map_to_odom,
        static_tf_odom_to_basefootprint,
        planner_server,
        controller_server,
        lifecycle_manager
    ])
