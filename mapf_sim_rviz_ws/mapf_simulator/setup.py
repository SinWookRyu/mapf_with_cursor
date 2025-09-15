from setuptools import find_packages, setup

package_name = 'mapf_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mapf_simulator.launch.py', 'launch/mapf_simulator_fixed.launch.py']),
        ('share/' + package_name + '/config', ['config/mapf_simulator.rviz', 'config/mapf_simulator_fixed.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mnerd',
    maintainer_email='mnerd@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapf_simulator_node = mapf_simulator.mapf_simulator_node:main',
            'gui_control_node = mapf_simulator.gui_control_node:main',
            'simple_test_node = mapf_simulator.simple_test_node:main',
            'simple_mapf_node = mapf_simulator.simple_mapf_node:main',
        ],
    },
)
