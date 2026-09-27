from setuptools import find_packages, setup

package_name = 'orchard_navigation'

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
    description='Orchard row following and lane-by-lane mission',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'row_navigator_node = orchard_navigation.row_navigator_node:main',
            'orchard_mapper_node = orchard_navigation.orchard_mapper_node:main',
        ],
    },
)
