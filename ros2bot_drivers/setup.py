from setuptools import setup

package_name = 'ros2bot_drivers'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abcampbellb',
    maintainer_email='abcampbellb@gmail.com',
    description='ros2bot drivers package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'master_driver_node = ros2bot_drivers.ros2bot_master_driver_node:main',
            'speach_driver_node = ros2bot_drivers.ros2bot_speach_driver_node:main',
            'master_speach_driver_node = ros2bot_drivers.ros2bot_master_speach_driver_node:main'
        ],
    },
)
