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
    maintainer='turtle',
    maintainer_email='turtle@todo.todo',
    description='final projet',
    license='Apach-2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'projet = challange_one.challange_one_node:main',
        ],
    },
)
