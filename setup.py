from glob import glob
import os
from setuptools import setup

package_name = 'ros2_rtimulib'
share_dir = os.path.join('share', package_name)

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join(share_dir, 'launch'), glob('launch/*.launch.py')),
        (os.path.join(share_dir, 'config'), glob('config/*.yaml')),
        (share_dir, glob('config/*.ini')),
    ],
    install_requires=['RTIMU',
                      'pyyaml'],
    zip_safe=True,
    author='Kurt Kiefer',
    maintainer='Kurt Kiefer',
    keywords=['ROS2'],
    maintainer_email='kekiefer@gmail.com',
    description='A package to publish IMU messages from an rtimulib-compatible sensor.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = ros2_rtimulib.imu_node:main'
        ],
    },
)
