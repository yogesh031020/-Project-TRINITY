from setuptools import find_packages, setup

package_name = 'trinity_swarm'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/swarm_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yogesh E S',
    maintainer_email='yogesh031020@gmail.com',
    description='Project TRINITY: Autonomous Decentralized Multi-UAV Swarm Coordination & Self-Healing Controller for PX4 and Gazebo SITL',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swarm_commander = trinity_swarm.swarm_commander_node:main',
            'uav_controller = trinity_swarm.uav_swarm_controller:main',
        ],
    },
)
