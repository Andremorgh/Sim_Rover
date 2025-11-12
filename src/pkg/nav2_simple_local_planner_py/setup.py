from setuptools import setup
package_name = 'nav2_simple_local_planner_py'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/params', ['params/local_planner_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/local_planner_py.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Python local planner node for a differential-drive rover',
    license='Apache-2.0',
    entry_points={'console_scripts': ['simple_local_planner = nav2_simple_local_planner_py.simple_local_planner_node:main']},
)