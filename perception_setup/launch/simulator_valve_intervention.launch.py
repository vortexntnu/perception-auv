"""Simulator -> YOLO OBB -> Valve Detection.

Simulator variant of valve_intervention.launch.py. Instead of running a
RealSense camera and image_undistort, this launch file accepts the color
image topic, camera info topic, depth topics, and image dimensions directly
as launch arguments — allowing it to subscribe to topics published by the
simulator (or any other upstream source).

Pipeline:
1. Simulator publishes color image, camera info, depth image and depth info
2. YOLO OBB detects oriented bounding boxes on the color image
3. Valve Detection uses oriented bounding boxes + depth image to compute valve pose
"""

import os

import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

CONVERTED_IMAGE_TOPIC = '/yolo_obb/internal/converted_image'
ENCODER_RESIZE_TOPIC = '/yolo_obb_encoder/internal/resize/image'
TENSOR_OUTPUT_TOPIC = '/yolo_obb/tensor_pub'
TENSOR_INPUT_TOPIC = '/yolo_obb/tensor_sub'
DNN_IMAGE_ENCODER_NAMESPACE = 'yolo_obb_encoder/internal'


def _launch_setup(context, *args, **kwargs):
    """Setup launch description with all nodes."""
    pkg_dir = get_package_share_directory('perception_setup')

    yolo_config_path = os.path.join(pkg_dir, 'config', 'yolo', 'yolo_obb.yaml')
    with open(yolo_config_path) as f:
        yolo_cfg = yaml.safe_load(f)

    models_dir = os.path.join(pkg_dir, 'models')
    model_file_path = os.path.join(models_dir, 'obb_best_simulator.onnx')
    engine_file_path = os.path.join(models_dir, 'obb_best_simulator.engine')

    # Resolve launch args
    color_image_topic = context.launch_configurations['color_image_topic']
    camera_info_topic = context.launch_configurations['camera_info_topic']
    depth_image_topic = context.launch_configurations['depth_image_topic']
    depth_camera_info_topic = context.launch_configurations['depth_camera_info_topic']
    input_image_width = int(context.launch_configurations['image_width'])
    input_image_height = int(context.launch_configurations['image_height'])

    # YOLO OBB Pipeline
    image_format_converter = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_converter',
        parameters=[
            {
                'encoding_desired': yolo_cfg['encoding_desired'],
                'image_width': input_image_width,
                'image_height': input_image_height,
                'input_qos': 'sensor_data',
            }
        ],
        remappings=[
            ('image_raw', color_image_topic),
            ('image', CONVERTED_IMAGE_TOPIC),
        ],
    )

    tensor_rt_node = ComposableNode(
        name='tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[
            {
                'model_file_path': model_file_path,
                'engine_file_path': engine_file_path,
                'output_binding_names': yolo_cfg['output_binding_names'],
                'output_tensor_names': yolo_cfg['output_tensor_names'],
                'input_tensor_names': yolo_cfg['input_tensor_names'],
                'input_binding_names': yolo_cfg['input_binding_names'],
                'verbose': bool(yolo_cfg['verbose']),
                'force_engine_update': bool(yolo_cfg['force_engine_update']),
                'tensor_output_topic': TENSOR_OUTPUT_TOPIC,
            }
        ],
        remappings=[
            ('tensor_pub', TENSOR_OUTPUT_TOPIC),
            ('tensor_sub', TENSOR_INPUT_TOPIC),
        ],
    )

    yolo_obb_decoder_node = ComposableNode(
        name='yolo_obb_decoder_node',
        package='isaac_ros_yolov26_obb',
        plugin='nvidia::isaac_ros::yolov26_obb::YoloV26OBBDecoderNode',
        parameters=[
            {
                'tensor_input_topic': TENSOR_INPUT_TOPIC,
                'confidence_threshold': float(yolo_cfg['confidence_threshold']),
                'num_detections': int(yolo_cfg['num_detections']),
                'detections_topic': str(yolo_cfg['detection_topic']),
            }
        ],
    )

    tensor_rt_container = ComposableNodeContainer(
        name='obb_tensor_rt_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            image_format_converter,
            tensor_rt_node,
            yolo_obb_decoder_node,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        namespace='',
    )

    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')
    dnn_image_encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')
        ),
        launch_arguments={
            'input_image_width': str(input_image_width),
            'input_image_height': str(input_image_height),
            'network_image_width': str(yolo_cfg['network_image_width']),
            'network_image_height': str(yolo_cfg['network_image_height']),
            'image_mean': str(yolo_cfg['image_mean']),
            'image_stddev': str(yolo_cfg['image_stddev']),
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'obb_tensor_rt_container',
            'dnn_image_encoder_namespace': DNN_IMAGE_ENCODER_NAMESPACE,
            'image_input_topic': CONVERTED_IMAGE_TOPIC,
            'camera_info_input_topic': camera_info_topic,
            'tensor_output_topic': TENSOR_OUTPUT_TOPIC,
        }.items(),
    )

    yolo_obb_visualizer = (
        Node(
            package='isaac_ros_yolov26_obb',
            executable='isaac_ros_yolov26_obb_visualizer.py',
            name='yolo_obb_visualizer',
            parameters=[
                {
                    'detections_topic': yolo_cfg['detection_topic'],
                    'image_topic': ENCODER_RESIZE_TOPIC,
                    'output_image_topic': yolo_cfg['visualized_image_topic'],
                    'class_names_yaml': str(yolo_cfg['class_names']),
                }
            ],
        )
        if bool(yolo_cfg['enable_visualizer'])
        else None
    )

    # Valve Detection
    valve_detection_config = os.path.join(
        get_package_share_directory('valve_detection'),
        'config',
        'valve_detection_params.yaml',
    )

    valve_detection_container = ComposableNodeContainer(
        name='valve_detection_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='valve_detection',
                plugin='valve_detection::ValvePoseNode',
                name='valve_pose_node',
                parameters=[
                    valve_detection_config,
                    {
                        'depth_image_sub_topic': depth_image_topic,
                        'detections_sub_topic': yolo_cfg['detection_topic'],
                        'depth_image_info_topic': depth_camera_info_topic,
                        'depth_frame_id': 'front_camera_depth_optical',
                        'color_frame_id': 'front_camera_color_optical',
                        'landmarks_pub_topic': '/valve_landmarks',
                        'output_frame_id': 'front_camera_depth_optical',
                        'drone': LaunchConfiguration('drone'),
                        'undistort_detections': LaunchConfiguration(
                            'undistort_detections'
                        ),
                        'debug_visualize': LaunchConfiguration('debug_visualize'),
                    },
                ],
            )
        ],
        output='screen',
    )

    image_to_gstreamer_node = Node(
        package='image_to_gstreamer',
        executable='image_to_gstreamer_node',
        name='image_to_gstreamer_node',
        additional_env={'EGL_PLATFORM': 'surfaceless'},
        parameters=[
            {
                'input_topic': yolo_cfg['visualized_image_topic'],
                'host': '10.0.0.169',
                'port': 5000,
                'bitrate': 500000,
                'framerate': 15,
                'preset_level': 1,
                'iframe_interval': 15,
                'control_rate': 1,
                'pt': 96,
                'config_interval': 1,
                'format': 'RGB',
            }
        ],
        output='screen',
    )

    # Collect all actions
    actions = [
        tensor_rt_container,
        dnn_image_encoder_launch,
        valve_detection_container,
        image_to_gstreamer_node,
    ]

    if yolo_obb_visualizer:
        actions.append(yolo_obb_visualizer)

    return actions


def generate_launch_description():
    """Generate the launch description."""
    return launch.LaunchDescription(
        [
            # Simulator camera topic/size inputs
            DeclareLaunchArgument(
                'color_image_topic',
                default_value='/nautilus/front_camera/image_color',
                description='Color image topic published by the simulator',
            ),
            DeclareLaunchArgument(
                'camera_info_topic',
                default_value='/nautilus/front_camera/camera_info',
                description='Camera info topic for the color image',
            ),
            DeclareLaunchArgument(
                'depth_image_topic',
                default_value='/nautilus/depth_camera/image_depth',
                description='Depth image topic published by the simulator',
            ),
            DeclareLaunchArgument(
                'depth_camera_info_topic',
                default_value='/nautilus/depth_camera/camera_info',
                description='Camera info topic for the depth image',
            ),
            DeclareLaunchArgument(
                'image_width',
                default_value='1920',
                description='Input color image width in pixels',
            ),
            DeclareLaunchArgument(
                'image_height',
                default_value='1080',
                description='Input color image height in pixels',
            ),
            # Valve Detection Options
            DeclareLaunchArgument(
                'drone',
                default_value='nautilus',
                description='Robot name, prepended to TF frame IDs',
            ),
            DeclareLaunchArgument(
                'undistort_detections',
                default_value='false',
                description='Undistort detections using color camera distortion',
            ),
            DeclareLaunchArgument(
                'debug_visualize',
                default_value='true',
                description='Enable debug visualization topics',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
