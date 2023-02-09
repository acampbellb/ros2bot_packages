# ros2bot base cpp package

This package contains the base node for the robot.

## Creating the package

1.  Create the package

    ```
    $ cd ~/ros2bot_ws/src
    $ ros2 pkg create ros2bot_base --build-type ament_cmake
    ```

    The directory structure should look like this now.

    ```
    ros2bot_base/
    ├── CMakeLists.txt
    ├── include
    │   └── ros2bot_base
    ├── package.xml
    └── src
    ```

2.  Build the package

    ```
    $ cd ~/ros2bot_ws
    $ colcon build
    ```

## Building node inside the package

1.  Create a cpp file inside the src of the package.

    ```
    $ cd ~/ros2bot_ws/src/ros2bot_base/src/
    $ touch ros2bot_base_node.cpp
    ```

2.  Write the node source (refer to source files)



