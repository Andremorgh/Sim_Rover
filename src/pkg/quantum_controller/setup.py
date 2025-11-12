from setuptools import setup

package_name = 'quantum_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/quantum_controller.launch.py']),
        ('share/' + package_name + '/config', ['config/LUT_lin.csv']),
        ('share/' + package_name + '/config', ['config/LUT_ang.csv']),
        ('share/' + package_name + '/launch', ['launch/quantum_system.launch.py']),
        ('share/' + package_name + '/launch', ['launch/quantum_system_sim.launch.py']),
        ('share/' + package_name + '/launch', ['launch/nav2_rover.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/quantum_goal.launch.py']),
        ('share/' + package_name + '/config', ['config/LUT_POS_X.csv']),
        ('share/' + package_name + '/config', ['config/LUT_POS_Y.csv']),
    ],
    install_requires=['setuptools', 'numpy',"scipy"],
    zip_safe=True,
    maintainer='Andrea',
    maintainer_email='andrea@example.com',
    description='Controllore ROS2 con LUT fisse per rover differenziale',
    license='MIT',
    entry_points={
    'console_scripts': [
        'quantum_controller_node = quantum_controller.quantum_controller_node:main',
        'lidar_listener = quantum_controller.lidar_listener:main',
        'quantum_controller_node_sim = quantum_controller.quantum_controller_node_sim:main',
        'lidar_listener_sim = quantum_controller.lidar_listener_sim:main',    
        'quantum_goal_generator = quantum_controller.quantum_goal_generator:main',  
        ],
    },
)
