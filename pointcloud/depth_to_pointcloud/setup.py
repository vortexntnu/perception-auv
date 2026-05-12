from setuptools import setup
from glob import glob
import os

package_name = 'depth_to_pointcloud'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kluge7',
    maintainer_email='asvendsr@gmail.com',
    description='Convert depth images to point clouds for ROS2 Humble.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'depth_to_pointcloud_node = depth_to_pointcloud.depth_to_pointcloud_node:main',
            'depth_cropper_node = depth_to_pointcloud.depth_cropper_node:main',
            'sync_and_save_node = depth_to_pointcloud.sync_and_save_node:main',
        ],
    },
)
