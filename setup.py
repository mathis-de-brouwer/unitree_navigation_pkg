from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'unitree_navigation_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='senne',
    maintainer_email='senne.clauwaert@student.ehb.be',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'navigation_node = unitree_navigation_pkg.navigation_node:main',
                'segmentation_node = unitree_navigation_pkg.segmentation_node:main', 
                # 'websocket_server = unitree_navigation_pkg.websocket_server:main'         
        ],
    },
)
