from setuptools import setup

package_name = 'radar_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Group4',
    maintainer_email='group4@example.com',
    description='Camera node for RADAR',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = radar_camera.camera_node:main',
        ],
    },
)

