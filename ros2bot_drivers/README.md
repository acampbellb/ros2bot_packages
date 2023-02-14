# ros2bot drivers package

The driver nodes for the robot.

## Package composition

1.  Create the package

    ```
    $ cd ~/ros2bot_ws/src/
    $ ros2 pkg create ros2bot_drivers --build-type ament_python 
    ```

    The drivers package contents should look like this:

    ```
    ros2bot_drivers/
    ├── ros2bot_drivers
    │   └── __init__.py
    ├── package.xml
    ├── resource
    │   └── ros2bot_drivers
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

## Compose driver nodes within the package

Build the driver nodes within the package.

1.  Create master driver node

    ```
    $ cd ~/ros2bot_ws/src/ros2bot_drivers/ros2bot_drivers/
    $ touch ros2bot_master_driver_node.py
    ```

2.  Write the node code in the new python file (see source code)

3.  Do same for speach driver node.

4.  Add entrypoints in setup.py

    ```
    entry_points={
        'console_scripts': [
            'master_driver_node = ros2bot_drivers.ros2bot_master_driver_node:main',
            'speach_driver_node = ros2bot_drivers.ros2bot_speach_driver_node:main'
        ],
    },
    ```

    master_driver_node & speach_driver_node are the names of the executables of the nodes.

5.  Add dependencies to package.xml

