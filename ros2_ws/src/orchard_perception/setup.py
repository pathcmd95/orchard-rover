from setuptools import find_packages, setup

package_name = 'orchard_perception'

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
    description='Orchard perception: trunk/row detection from 3D LiDAR, camera YOLO, fusion',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_tree_node = orchard_perception.lidar_tree_node:main',
            'camera_tree_node = orchard_perception.camera_tree_node:main',
            'tree_fusion_node = orchard_perception.tree_fusion_node:main',
            'seg_path_node = orchard_perception.seg_path_node:main',
            'seg_dataset_node = orchard_perception.seg_dataset_node:main',
        ],
    },
)
