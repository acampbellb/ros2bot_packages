# ros2bot robot controls package

This package contains control nodes for the ros2bot robot.

## Package composition

1.  Create the package

    ```
    $ cd ~/ros2bot_ws/src/
    $ ros2 pkg create ros2bot_controls --build-type ament_python 
    ```

    The drivers package contents should look like this:

    ```
    ros2bot_controls/
    ├── ros2bot_controls
    │   └── __init__.py
    ├── package.xml
    ├── resource
    │   └── ros2bot_controls
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

## Compose control nodes within the package