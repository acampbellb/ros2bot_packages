import os

from glob import glob
from setuptools import setup

package_name = 'ros2bot_calibrate'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abcampbellb',
    maintainer_email='acampbellb@gmail.com',
    description='ros2bot calibration package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrate_angular_node = ros2bot_calibrate.calibrate_angular:main',
            'calibrate_linear_node = ros2bot_calibrate.calibrate_linear:main'
        ],
    },
)
