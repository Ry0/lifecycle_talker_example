from setuptools import setup
import os
from glob import glob

package_name = 'lifecycle_talker_example'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryo Kabutan',
    maintainer_email='kabutan.github@gmail.com',
    description='ROS2 Jazzy lifecycle node example',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lifecycle_talker = lifecycle_talker_example.lifecycle_talker:main',
            'lifecycle_subscriber = lifecycle_talker_example.lifecycle_subscriber:main',
        ],
    },
)
