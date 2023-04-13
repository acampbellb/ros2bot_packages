# ros2bot camera package

This package contains nodes for the ZED 2i stereo depth camera.

# Prerequisites

## Install Python ZED SDK

To install the Python ZED SDK, follow these [directions](https://www.stereolabs.com/docs/app-development/python/install/).

## Install ZED ROS2 wrapper package

To install the zed_ros2_wrapper, open a bash terminal, clone the package from Github, and build it:

```
$ cd ~/ros2_ws/src/ #use your current ros 2 workspace folder
$ git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
$ cd ..
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
$ echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
$ source ~/.bashrc
```

**Note:** The zed-ros2-wrapper repository contains the repository zed-ros2-interfaces as a sub-module. zed-ros2-interfaces contains the definitions of the custom topics and services, and the meshes for the 3D visualization of the camera models on rviz 2. It is very important to use the command --recursive while cloning the repository to retrieve also the latest version of the sub-module repository.

**Note:** The option --symlink-install is very important. It allows to use symlinks instead of copying files to the ROS 2 folders during the installation, where possible. Each package in ROS 2 must be installed and all the files used by the nodes must be copied into the installation folders. Using symlinks allows you to modify them in your workspace, reflecting the modification during the next executions without the needing to issue a new colcon build command. This is true only for all the files that don’t need to be compiled (Python launch scripts, YAML configurations, etc).

# Starting ZED node

To start the ZED node, open a terminal and use the CLI command ros2 launch:

```
$ ros2 launch zed_wrapper zed2i.launch.py
```

The zed2i.launch.py is a Python script that automatically start the ZED node using “manual composition”, loading the parameters from the correct “YAML files”, and creating the camera model from the “URDF file” that is automatically created from a xacro configuration file.

**Note:** You can set your own configurations by modifying the parameters in the files common.yaml, zed2i.yaml  available in the folder zed_wrapper/config. For full descriptions of each available parameter, follow the [complete guide](https://www.stereolabs.com/docs/ros2/zed-node/#configuration-parameters).

# Displaying ZED data

## Using RVIZ 2

RVIZ 2 is a useful visualization tool in ROS 2. Using rviz 2, you can visualize the left and right images acquired by the ZED cameras, the depth image and the 3D colored point cloud, plus other useful information.

Launch the ZED wrapper along with rviz 2 by using the following command (installing the [zed-ros2-examples](https://github.com/stereolabs/zed-ros2-examples) repository is required):

```
$ roslaunch zed_display_rviz display_zed2i.launch.py
```

**Note:** If you haven’t yet configured your own rviz 2 interface, you can find a detailed tutorial [here](https://www.stereolabs.com/docs/ros2/rviz2/).


## Displaying Images

The ZED node publishes both original and stereo rectified (aligned) left and right images. In rviz 2, select a topic and use the image preview mode.

Here is a list of the main available image topics (more image topics are [available](https://www.stereolabs.com/docs/ros2/zed-node/#published-topics)):

- rgb/image_rect_color: Color rectified image (left image by default)
- rgb/image_raw_color: Color unrectified image (left image by default)
- right/image_rect_color: Color rectified right image
- right/image_raw_color: Color unrectified right image
- left/image_rect_color: Color rectified left image
- left/image_raw_color: Color unrectified left image
- confidence/confidence_image: Confidence map

## Displaying Depth

The depth map can be displayed in RVIZ subscribing to the following topic:

depth/depth_registered: 32-bit depth values in meters. RVIZ will normalize the depth map on 8-bit and display it as a grayscale depth image.

**Note:** An OpenNI compatibility mode is available modifying the config/common.yaml file. Set depth.openni_depth_mode to true to get depth in millimeters with 16-bit precision, then restart the ZED node.

## Displaying the Point Cloud

A 3D colored point cloud can be displayed in rviz 2 subscribing to the point_cloud/cloud_registered topic.

Add it in rviz 2 with point_cloud -> cloud -> PointCloud2. Note that displaying point clouds slows down rviz 2, so open a new instance if you want to display other topics.

## Displaying Position and Path

The ZED position and orientation in space over time is published to the following topics:

- odom: Odometry pose referred to odometry frame (only visual odometry is applied for ZED, visual-inertial for ZED Mini)
- pose: Camera pose referred to Map frame (complete data fusion algorithm is applied)
- pose_with_covariance: Camera pose referred to Map frame with covariance (if spatial_memory is false in launch parameters)
- path_odom: The sequence of camera odometry poses in Map frame
- path_map: The sequence of camera poses in Map frame

# Launching with a recorded SVO video

With the ZED, you can record and play back stereo video using Stereolabs' .SVO file format. To record a sequence, open the ZED Explorer app (/usr/local/zed/tools) and click on the REC button.

To launch the ROS wrapper with an SVO file, set the relative parameter while starting the node. For example, to start the ZED 2 node using the SVO as input source:

```
$ ros2 launch zed_wrapper zed2i.launch.py svo_path:=<full_path_to_svo_file>
```

**Important:** Use only full paths to the SVO file. Relative paths are not allowed.

# Dynamic reconfigure

You can dynamically change many configuration parameters during the execution of the ZED node. All the parameters that can be dynamically changed while the node is running are marked as [DYNAMIC] in the YAML configuration files. For a full description of each parameter please read the [complete guide](https://www.stereolabs.com/docs/ros2/zed-node#configuration-parameters).

You can set the parameters by using the CLI command ros2 param set, e.g.:

```
$ ros2 param set /zed2/zed_node depth.depth_confidence 80
```

if the parameter is successfully set you will get a confirmation message:

```
Set parameter successful
```

If you try to set a parameter that’s not dynamically reconfigurable, or if you provided an invalid value, you will get this error:

```
$ ros2 param set /zed2/zed_node depth.depth_confidence 150
Setting parameter failed: depth.depth_confidence must be a positive integer in the range [0,100]
```

You can also using a GUI to set dynamic parameters values:

```
$ rqt
```

then select Plugins -> Configuration -> Dynamic Reconfigure

# Node Diagnostic

The ZED ROS 2 node publishes useful diagnostic information aggregated into the /diagnostics topic. Diagnostic information can be analyzed and parsed by using ROS 2 tools, like for example the Runtime Monitor plugin of rqt

# Use OpenCV with ZED in Python

How to capture and display color and depth images using OpenCV and the ZED SDK in Python.

Sample code is available on [GitHub](https://github.com/stereolabs/zed-opencv/tree/master/python). Make sure the [ZED Python API](https://github.com/stereolabs/zed-python-api) is installed before launching the sample.

## Sharing image data between ZED SDK and OpenCV Python

In Python, OpenCV store images in NumPy arrays. Since the ZED SDK uses its own sl.Mat class to store image data, we provide a function get_data() to convert the sl.Mat matrix into a NumPy array.

```
# Create an RGBA sl.Mat object
image_zed = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)
# Retrieve data in a numpy array with get_data()
image_ocv = image_zed.get_data()
```

## Capturing Video

A depth map is a 1-channel matrix with 32-bit float values for each pixel. Each value expresses the distance of a pixel in the scene. The depth map can be retrieved using retrieve_measure() and loaded with get_data() into a NumPy array. Please refer to the [Depth API](https://www.stereolabs.com/docs/depth-sensing/using-depth/) for more information.

```
# Create a sl.Mat with float type (32-bit)
depth_zed = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.F32_C1)

if zed.grab() == sl.ERROR_CODE.SUCCESS :
    # Retrieve depth data (32-bit)
    zed.retrieve_measure(depth_zed, sl.MEASURE.DEPTH)
    # Load depth data into a numpy array
    depth_ocv = depth_zed.get_data()
    # Print the depth value at the center of the image
    print(depth_ocv[int(len(depth_ocv)/2)][int(len(depth_ocv[0])/2)])
```

## Displaying Depth

A NumPy array with 32-bit float values can’t be displayed with cv2.imshow. To display the depth map, we need to normalize the depth values between 0 and 255 (8-bit) and create a black and white representation. Do not use this representation for other purposes than displaying the image.

```
# Create an RGBA sl.Mat object
image_depth_zed = sl.Mat(zed.get_camera_information().camera_resolution.width, sl.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)

if zed.grab() == SUCCESS :
    # Retrieve the normalized depth image
    zed.retrieve_image(image_depth_zed, sl.VIEW.DEPTH)
    # Use get_data() to get the numpy array
    image_depth_ocv = image_depth_zed.get_data()
    # Display the depth view from the numpy array
    cv2.imshow("Image", image_depth_ocv)
```

## UVC Capture

You can also use the ZED as a standard UVC camera in OpenCV to capture raw stereo video using the code snippet below. To get rectified images and calibration with OpenCV, use the native (Python) capture sample available on [GitHub](https://github.com/stereolabs/zed-opencv-native).

```
import cv2
import numpy

# Open the ZED camera
cap = cv2.VideoCapture(0)
if cap.isOpened() == 0:
    exit(-1)

# Set the video resolution to HD720 (2560*720)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True :
    # Get a new frame from camera
    retval, frame = cap.read()
    # Extract left and right images from side-by-side
    left_right_image = numpy.split(frame, 2, axis=1)
    # Display images
    cv2.imshow("frame", frame)
    cv2.imshow("right", left_right_image[0])
    cv2.imshow("left", left_right_image[1])
    if cv2.waitKey(30) >= 0 :
        break

exit(0)
```

## Object Detection

See this [tutorial](https://github.com/stereolabs/zed-sdk/tree/master/tutorials/tutorial%206%20-%20object%20detection/python)



