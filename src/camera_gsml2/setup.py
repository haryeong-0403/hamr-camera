from setuptools import setup

package_name = 'camera_gsml2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='Camera publisher using OpenCV and cv_bridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_pub1 = camera_gsml2.camera_pub1:main',
            'camera_fps0 = camera_gsml2.camera_fps0:main',
            'camera_fps3 = camera_gsml2.camera_fps3:main',
            'camera_fps1 = camera_gsml2.camera_fps1:main',
            'camera_test = camera_gsml2.camera_test:main',
        ],
    },
)