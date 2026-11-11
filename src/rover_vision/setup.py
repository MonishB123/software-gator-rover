import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rover_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gator Rover Software Team',
    maintainer_email='swe@gatorrover.org',
    description='AprilTag detection for Gator Rover.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'apriltag_node = rover_vision.apriltag_node:main',
            'video_publisher = rover_vision.video_publisher_node:main',
        ],
    },
)