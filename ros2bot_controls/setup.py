from setuptools import setup

package_name = 'ros2bot_controls'

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
    description='ros2bot robot controllers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_node = ros2bot_controls.ros2bot_joy_node:main'
        ],
    },
)
