from glob import glob

from setuptools import find_packages, setup

package_name = 'orchard_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UNIST BTS 유니주행차',
    maintainer_email='pathcmd@gmail.com',
    description='Orchard world generator and Gazebo helpers',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_orchard = orchard_gazebo.generate:main',
            'lane_error_monitor = orchard_gazebo.lane_error_monitor:main',
        ],
    },
)
