from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'projet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='selene',
    maintainer_email='selene@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower = projet.line_following_node:main',
            'obstacle_avoidance = projet.obstacle_avoidance:main',
            'corridor_node = projet.corridor_node:main',
            'human_motion_control = projet.human_motion_control:main',
            'node_monitor = projet.node_monitor:main',
        ],
    },
)
