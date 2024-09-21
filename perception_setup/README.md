# Perception AUV Launch Instructions

This repository contains a launch file for running various nodes related to image processing, ArUco detection, and camera drivers in a ROS 2 environment. The launch file supports both composable nodes and regular nodes based on user-specified configurations.

## Prerequisites

Ensure that you have the following packages installed and properly set up in your ROS 2 workspace:
- `perception_setup`
- `image_filtering`
- `aruco_detector`
- `spinnaker_camera_driver`

## Launch File Description

The main launch file provided is `perception.launch.py`, which includes several arguments to control the behavior of the nodes being launched.

### Launch Arguments

- `enable_filtering` (default: `True`): Enable or disable the image filtering node.
- `enable_aruco` (default: `True`): Enable or disable the ArUco detection node.
- `enable_gripper_camera` (default: `True`): Enable or disable the gripper camera driver node.
- `enable_front_camera` (default: `True`): Enable or disable the front camera driver node.
- `enable_composable_nodes` (default: `True`): Enable or disable the use of composable nodes.

### Configuration Files

The following configuration files are used by the nodes:
- `image_filtering_params.yaml` Ros2 parameters for the image_filtering_node
- `aruco_detector_params.yaml` Ros2 parameters for the aruco_detector_node
- `gripper_camera_params.yaml` Ros2 parameters for the gripper_camera_node
- `gripper_camera_calib.yaml` Camera calibration file for the gripper_camera
- `front_camera_params.yaml` Ros2 parameters for the front_camera_node
- `front_camera_calib.yaml` Camera calibration file for the front_camera
- `blackfly_s_params.yaml` This file maps the ros parameters to the corresponding Spinnaker "nodes" in the camera.

These files should be located in the `config` directory of the `perception_setup` package.

## Usage

To use the launch file, follow these steps:

1. **Navigate to your ROS 2 workspace**:
    ```sh
    cd ~/<ros2_ws>
    ```

2. **Source your workspace**:
    ```sh
    source install/setup.bash
    ```

3. **Run the launch file with default arguments**:
    ```sh
    ros2 launch perception_setup perception.launch.py
    ```

4. **Run the launch file with custom arguments**:
    You can customize the launch arguments directly from the command line. For example, to disable the front camera and ArUco detection, use:
    ```sh
    ros2 launch perception_setup perception.launch.py enable_front_camera:=False enable_aruco:=False
    ```
5. **Launch with --show-args to print out all available launch arguments**
   ```sh
    ros2 launch perception_setup perception.launch.py --show-args
    ```
## Nodes and Composable Nodes

The launch file can run nodes as either composable nodes or as separate nodes based on the `enable_composable_nodes` argument.

### Composable Nodes

When `enable_composable_nodes` is set to `True`, the enabled nodes are launched as composable nodes within a container. To allow for intra-process-communication between the different composable nodes within the same container set the `use_intra_process_comms` argument to true for the individual nodes. 

For understanding of how to achieve a zero-copy transport of messages when publishing and subscribing in ros2 see the [Setting up efficient intra-process communication](https://docs.ros.org/en/humble/Tutorials/Demos/Intra-Process-Communication.html) tutorial.

When launching as composable nodes in the same container if one node crashes it will cause the other nodes in the same container to crash as well. To avoid this and achieve better fault isolation one can set the `enable_composable_nodes` to `False`.

### Standalone Nodes

When `enable_composable_nodes` is set to `False`, the enabled nodes will be launched as standalone nodes to allow for better fault isolation. The config files included in this repository will be still be used to create node instances directly in this launch file to provide a simplistic overview. This configuration also makes this launch file independent from the launch files in the other packages.

## Calibration Files

The camera calibration files are located in the `config` directory of the `perception_setup` package:
- `gripper_camera_calib.yaml`
- `front_camera_calib.yaml`

These files are referenced in the launch file to provide calibration data for the cameras.

## Example Commands

**Launch with all nodes enabled (default):**
```sh
ros2 launch perception_setup perception.launch.py
```
**Launch with only the gripper camera and image filtering enabled:**
```sh
ros2 launch perception_setup perception.launch.py enable_front_camera:=False enable_aruco:=False
```
**Launch without using composable nodes:**
```sh
ros2 launch perception_setup perception.launch.py enable_composable_nodes:=False

```