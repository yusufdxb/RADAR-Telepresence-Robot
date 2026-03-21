from setuptools import setup
import os
from glob import glob

package_name = 'radar_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name] if os.path.isdir(package_name) else [],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.isfile('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yusuf Guenena',
    maintainer_email='yusuf.a.guenena@gmail.com',
    description='RADAR bringup — launch files and system configuration',
    license='MIT',
    entry_points={'console_scripts': []},
)
