# ros2bot bringup launch package

The ros2bot bringup package contains launch implementations that start multiple nodes from different packages.

## Package composition

1.  Create bringup package and base launch

    In the new package, remove the src/ and include/ folders. Create a new launch/ folder, and create the first 
    launch file inside it, base launch.

    ```
    $ cd ~/ros2bot_ws/src
    $ ros2 pkg create ros2bot_bringup
    $ cd ros2bot_bringup/
    $ rm -rf include/
    $ rm -rf src/
    $ mkdir launch
    $ touch launch/base.launch.py
    ```

2.  Compose the base launch file

    ```
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        ld = LaunchDescription()

        base_node = Node(
            package="ros2bot_base",
            executable="ros2bot_base_node",
        )

        master_node = Node(
            package="ros2bot_drivers",
            executable="master_driver_node"
        )

        # add nodes to launch description
        ld.add_action(base_node)
        ld.add_action(master_node)

        # return launch description
        return ld
    ```

## Install launch file

Install the base launch file into the bringup package.

1.  Add dependencies

    As the nodes being added to the launch files are different packages than the bringup one, we need to 
    add some dependencies. Go to the 'package.xml' of the bringup package, and add an “exec_depend” tag 
    for each dependency.

    ```
    <exec_depend>ros2bot_base</exec_depend>
    <exec_depend>ros2bot_drivers</exec_depend>
    ```

    If, later on, new nodes are added from other packages to the launch file, add the dependencies.

2.  Install launch file (to Cpp package)

    In the 'CMakeLists.txt' file add the following after 'find_package(ament_cmake REQUIRED)'

    ```
    install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
    )
    ```

## Run launch file

1.  First open a new terminal window

2.  Run the launch file

    ```
    $ ros2 launch ros2bot_bringup base.launch.py
    ```

## Customize nodes in launch file

There are several options to customize your nodes in the launch file if needed.

1.  Rename node

    You may want to rename a node, for example if you want to start 2 different nodes from the same executable.

    ```
    base_node = Node(
        package="ros2bot_base",
        executable="ros2bot_base_node",
        name="my_base_node"
    )
    ```

2.  Topic / Service remapping

    To remap a topic/service inside a node, add a remappings[] array inside the Node object, and add a tuple: 
    first value is the current name, second value is the new name.

    ```
    base_node = Node(
        package="ros2bot_base",
        executable="ros2bot_base_node",
        name="my_base_node",
        remappings=[
            ("orig_topic_name", "new_topic_name")
        ]
    )

    master_node = Node(
        package="ros2bot_drivers",
        executable="master_driver_node",
        name="my_master_driver".
        remappings=[
            ("orig_topic_name", "new_topic_name")
        ]
    )
    ```

    In this case, as we want the base and master nodes to be able to communicate, we have to rename the 
    topic on both sides.

    To add more remappings, simply add other tuples inside the remappings[] array.

3.  Parameters

    If we want to start a node with different parameter values, we need to set some parameters values.

    ```
    base_node = Node(
        package="ros2bot_base",
        executable="ros2bot_base_node",
        name="my_base_node",
        parameters=[
            {"param_a": 10},
            {"param_b": 20},
            {"param_c": 30}
        ]
    )

    ```

    To add parameters, create a parameters[] array inside the node. For each parameter you want to set, 
    add a new dictionary: key is the parameter’s name, value is the parameter’s value.

    To set parameters in a launch file you can either set them directly like we did here, or load them 
    from a YAML config file, which may be more scalable when you start to have many parameters.

## Load a YAML config file for a node

As a ROS2 parameter only exist within a node, here is an example node with parameters.

The node by itself doesn’t (and doesn’t need to) know if the parameters where launched from a YAML file.

    ```
    import rclpy
    from rclpy.node import Node
    class TestYAMLParams(Node):
        def __init__(self):
            super().__init__('your_amazing_node')
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('bool_value', None),
                    ('int_number', None),
                    ('float_number', None),
                    ('str_text', None),
                    ('bool_array', None),
                    ('int_array', None),
                    ('float_array', None),
                    ('str_array', None),
                    ('bytes_array', None),
                    ('nested_param.another_int', None)
                ])
    def main(args=None):
        rclpy.init(args=args)
        node = TestYAMLParams()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    if __name__ == '__main__':
        main()
    ```

### Load from a launch file

    ```
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch_ros.actions import Node
    def generate_launch_description():
        ld = LaunchDescription()
        config = os.path.join(
            get_package_share_directory('ros2_tutorials'),
            'config',
            'params.yaml'
            )

        node=Node(
            package = 'ros2_tutorials',
            name = 'your_amazing_node',
            executable = 'test_yaml_params',
            parameters = [config]
        )
        ld.add_action(node)
        return ld
    ```

First we get the path of the YAML config file. In this example the file is expected to be in the 
“install/ros2_tutorials/share/ros2_tutorials/config/” folder of your ROS2 workspace.

Make sure the YMAL file is installed in the right place.

For a python package in 'setup.py':

    ```
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    ```

For a Cpp package, in 'CMakeLists.txt':

    ```
    install(DIRECTORY
      launch
      DESTINATION share/${PROJECT_NAME}
    )

    install(DIRECTORY
      config
      DESTINATION share/${PROJECT_NAME}
    )
    ```

 You can install the launch and config files wherever you want, the example above is a convention.

 Dump YAML file from existing params:

    ```
    $ ros2 param dump /your_amazing_node
    Saving to:  ./your_amazing_node.yaml
    ```

Or print directly the params on the terminal:

    ```
    $ ros2 param dump /your_amazing_node --print
    your_amazing_node:
      ros__parameters:
        bool_array:
        - true
        - false
        - true
        bool_value: true
        bytes_array: !!python/object/apply:array.array
        - q
        - - 1
          - 241
          - 162
        float_array: !!python/object/apply:array.array
        - d
        - - 7.5
          - 400.4
        float_number: 3.14
        int_array: !!python/object/apply:array.array
        - q
        - - 10
          - 11
          - 12
          - 13
        int_number: 5
        nested_param:
          another_int: 7
        str_array:
        - Nice
        - more
        - params
        str_text: Hello Universe
        use_sim_time: false
    ```













