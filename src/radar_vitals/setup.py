from setuptools import setup

package_name = 'radar_vitals'

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
    maintainer='Yusuf Guenena',
    maintainer_email='yusuf.a.guenena@gmail.com',
    description='Vital sign monitoring for RADAR (MAX30102 driver + simulator)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vitals_node   = radar_vitals.vitals_node:main',
            'pulse_ox_node = radar_vitals.pulse_ox_node:main',
        ],
    },
)
