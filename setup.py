from setuptools import setup
import os
from glob import glob

package_name = 'unitree_navigation_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
        (os.path.join('share', package_name, 'models'), glob('models/*.mp4')),  # Add this line to include mp4 files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = unitree_navigation_pkg.navigation_node:main',
            'segmentation_node = unitree_navigation_pkg.segmentation_node:main',
            'websocket_server = unitree_navigation_pkg.websocket_server:main',
            'video_navigation_tester = unitree_navigation_pkg.video_navigation_tester:main',
        ],
    },
)
