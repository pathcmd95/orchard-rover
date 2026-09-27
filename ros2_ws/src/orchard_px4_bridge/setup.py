from setuptools import find_packages, setup

package_name = 'orchard_px4_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UNIST BTS 유니주행차',
    maintainer_email='pathcmd@gmail.com',
    description='ROS 2 <-> PX4 rover bridge (offboard setpoints, odometry)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_node = orchard_px4_bridge.offboard_node:main',
            'state_node = orchard_px4_bridge.state_node:main',
        ],
    },
)
