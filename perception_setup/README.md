# perception_setup

Launch and configuration package for the perception pipeline. It contains camera drivers, image preprocessing, YOLO inference pipelines, and the mission launch files that tie everything together.

The main idea is:
- Root launch files are the mission-level entry points we actually use during operations.
- `launch/yolo/` contains the individual Isaac ROS YOLO launch files for standalone inference pipelines.
- `launch/ultralytics/` contains equivalent launch files built with Ultralytics nodes instead of Isaac ROS nodes.
- `launch/isaac_ros/` contains Isaac ROS variants similar to the Ultralytics ones, but they do not launch cameras themselves; they expect the camera topics to already exist.

Config YAMLs hold model and algorithm tuning only: thresholds, tensor names, model paths, class names, and similar parameters. Launch files own the wiring and topic names.

## Package structure

```
perception_setup/
  config/
    cameras/
      color_realsense_d555_calib.yaml     # RealSense D555 color camera calibration (K, D)
      blackfly_s_calib.yaml               # Blackfly S camera calibration
      blackfly_s_params.yaml              # Blackfly S ROS parameters
      blackfly_s_driver_params.yaml       # Spinnaker SDK node mapping
    yolo/
      yolo_obb.yaml                       # YOLO OBB tuning (model paths, thresholds, tensor names)
      yolo_detect.yaml                    # YOLO detection tuning
      yolo_seg.yaml                       # YOLO segmentation tuning
      yolo_cls.yaml                       # YOLO classification tuning
  launch/
    cameras/
      realsense_d555.launch.py            # RealSense D555 + image_undistort
      blackfly_s.launch.py                # Blackfly S camera driver
    isaac_ros/
      isaac_ros_valve_intervention.launch.py  # Simulator-driven Isaac ROS valve pipeline
    yolo/
      yolo_obb.launch.py                  # Standalone YOLO OBB inference
      yolo_detect.launch.py               # Standalone YOLO detection inference
      yolo_seg.launch.py                  # Standalone YOLO segmentation inference
      yolo_cls.launch.py                  # Standalone YOLO classification inference
    ultralytics/
      ultralytics_pipeline_line_fitting.launch.py  # Ultralytics-based pipeline
      ultralytics_valve_detection.launch.py         # Ultralytics valve pipeline
    valve_intervention.launch.py          # Full valve detection pipeline
    visual_inspection.launch.py           # ArUco marker detection pipeline
  models/                                 # ONNX and TensorRT engine files
  src/
    image_undistort.cpp                   # C++ composable node (lens undistortion)
  include/
    perception_setup/
      image_undistort.hpp
  scripts/
    image_undistort.py                    # Python equivalent (legacy, kept for reference)
    image_crop.py                         # Image cropping utility node
    camera_info_publisher.py              # Standalone camera_info publisher
```

## Helper nodes

This package provides one C++ composable node:

| Plugin name | Description |
|---|---|
| `perception_setup::ImageUndistort` | Undistorts a raw camera image using a calibration YAML, or passes through unchanged. Publishes a rectified image and zero-distortion `camera_info`. |

### Python scripts (not currently used in any launch file)

| Script | Description |
|---|---|
| `camera_info_publisher.py` | Publishes a `sensor_msgs/CameraInfo` message from a calibration YAML file on a given topic. Useful for cameras whose drivers do not publish camera_info. |
| `image_crop.py` | Crops an image and updates the corresponding camera_info. Was previously used for depth image cropping but removed because it interfered with valve detection. |

### ImageUndistort parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `image_topic` | string | *(required)* | Input raw image topic |
| `camera_info_topic` | string | `""` | Input camera_info topic (used if `camera_info_file` is empty) |
| `camera_info_file` | string | `""` | Path to calibration YAML (takes priority over topic) |
| `raw_camera_info_topic` | string | *(required)* | Raw camera_info topic (used in passthrough mode) |
| `output_image_topic` | string | *(required)* | Output rectified image topic |
| `output_camera_info_topic` | string | *(required)* | Output camera_info topic |
| `enable_undistort` | bool | *(required)* | `true` = undistort, `false` = passthrough |
| `image_qos` | string | `"sensor_data"` | QoS for image publisher: `"reliable"` or `"sensor_data"` (best effort) |

## Launch files

### valve_intervention.launch.py

Full valve detection pipeline used in missions: RealSense D555 -> image undistortion -> YOLO OBB -> valve pose estimation.

All C++ nodes run in composable containers for zero-copy intra-process transport:
- **obb_tensor_rt_container**: RealSense driver, image_undistort, image format converter, DNN image encoder, TensorRT, YOLO OBB decoder
- **valve_detection_container**: valve pose estimator

```sh
ros2 launch perception_setup valve_intervention.launch.py
```

| Argument | Default | Description |
|---|---|---|
| `enable_undistort` | `true` | Undistort color image before YOLO inference |
| `drone` | `nautilus` | Robot name, prepended to TF frame IDs |
| `undistort_detections` | `false` | Undistort YOLO detections using lens distortion coefficients (mutually exclusive with `enable_undistort`) |
| `debug_visualize` | `true` | Enable debug visualization topics |

### visual_inspection.launch.py

ArUco marker detection pipeline: RealSense D555 -> image undistortion -> image filtering -> ArUco detector.

All nodes run in a single composable container (`visual_inspection_container`).

```sh
ros2 launch perception_setup visual_inspection.launch.py
```

| Argument | Default | Description |
|---|---|---|
| `enable_undistort` | `true` | Undistort color image before processing |

### cameras/realsense_d555.launch.py

Standalone RealSense D555 camera driver with image undistortion. Publishes both color (undistorted) and depth streams.

```sh
ros2 launch perception_setup realsense_d555.launch.py
```

| Argument | Default | Description |
|---|---|---|
| `enable_undistort` | `true` | Undistort color image before publishing |

## Configuration

### YOLO config files

Each YOLO variant (`yolo_obb.yaml`, `yolo_detect.yaml`, etc.) holds model tuning only: model/engine paths, network input dimensions, tensor I/O bindings, confidence thresholds, class names, encoding, and visualizer enable. No topics.

### Launch layout summary

- Root launch files are the mission entry points.
- `yolo/` launch files are the standalone Isaac ROS inference pipelines.
- `ultralytics/` launch files use Ultralytics nodes for the same kinds of inference tasks.
- `isaac_ros/` launch files use Isaac ROS inference nodes without launching cameras.

## Building

```sh
colcon build --packages-up-to perception_setup
```

The C++ composable node requires: `rclcpp`, `rclcpp_components`, `sensor_msgs`, `cv_bridge`, `OpenCV`, `yaml-cpp`.
