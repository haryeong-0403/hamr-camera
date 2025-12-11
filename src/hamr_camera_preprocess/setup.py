from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hamr_camera_preprocess'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontcam_rectifier = hamr_camera_preprocess.frontcam_rectifier:main',
            'detect_image = hamr_camera_preprocess.detect_image:main',
            'lidar_camera_fusion = hamr_camera_preprocess.lidar_camera_fusion:main',
            'lidar_pointlcoud_sub = hamr_camera_preprocess.lidar_pointlcoud_sub:main',
            'gstreamer = hamr_camera_preprocess.gstreamer:main',
            'camera_selector_node = hamr_camera_preprocess.camera_selector_node:main',
        ],
    },
)