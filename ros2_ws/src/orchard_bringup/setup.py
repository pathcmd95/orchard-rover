from glob import glob

from setuptools import find_packages, setup

package_name = 'orchard_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UNIST BTS 유니주행차',
    maintainer_email='pathcmd@gmail.com',
    description='Launch files and parameters for the orchard rover',
    license='Apache-2.0',
)
