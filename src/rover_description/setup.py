from setuptools import setup
import os
from glob import glob

package_name = 'rover_description'

setup(
    name=package_name,
    version='0.0.0',
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        # Copy the urdf files into the share folder so that they can be found by the launch files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
    ],
    maintainer='Gator Rover Software Team',
    maintainer_email='swe@gatorrover.org',
    license='Apache-2.0',
    install_requires=['setuptools'],
    zip_safe=True,
    description='Rover Description Package',
)
