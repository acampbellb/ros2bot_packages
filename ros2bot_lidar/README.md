# ros2bot laser package

Laser package for ros2bot's Slamtec S2 RPLIDAR lidar.

## Package composition

1.  Create the package

    ```
    $ cd ~/ros2bot_ws/src/
    $ ros2 pkg create ros2bot_lidar --build-type ament_python 
    ```

    The drivers package contents should look like this:

    ```
    ros2bot_lidar/
    ├── ros2bot_lidar
    │   └── __init__.py
    ├── package.xml
    ├── resource
    │   └── ros2bot_lidar
    ├── setup.cfg
    ├── setup.py
    └── test
        ├── test_copyright.py
        ├── test_flake8.py
        └── test_pep257.py
    ```

2.  Update the package.xml and setup.py files with the appropriate values for the following fields:

    ```
    version
    description
    maintainer
    license

    ```

3.  Build the packages

    ```
    $ cd ~/ros2bot_ws
    $ colcon build
    ```

4.  Compose package nodes (avoidance, tracking, warning) see source code.

5.  Add entrypoints to 'setup.py'

    ```
    entry_points={
        'console_scripts': [
            'lidar_avoidance_node = ros2bot_lidar.ros2bot_lidar_avoidance_node:main',
            'lidar_tracking_node = ros2bot_lidar.ros2bot_lidar_tracking_node:main',
            'lidar_warning_node = ros2bot_lidar.ros2bot_lidar_warning_node:main'
        ],
    },
    ```
