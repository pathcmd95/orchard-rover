"""orchard_planning 패키지 설치 설정 (ament_python). 노드 실행 이름은 entry_points 참고."""
from setuptools import find_packages, setup

package_name = 'orchard_planning'

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
    description='Global (orchard coverage + Hybrid A*) and local (DWA) path planners',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_planner_node = orchard_planning.global_planner_node:main',
            'local_planner_node = orchard_planning.local_planner_node:main',
        ],
    },
)
