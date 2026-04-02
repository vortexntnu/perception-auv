"""RealSense D555 -> YOLO OBB -> Valve Detection.

Pipeline:
1. RealSense D555 camera publishes raw color image and rectified depth image
2. image_undistort undistorts the raw color image
3. YOLO OBB detects oriented bounding boxes on the undistorted color image
4. Valve Detection uses oriented bounding boxes and depth image detections to compute valve pose
"""

import os

import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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

    # Load configurations
    cameras_path = os.path.join(pkg_dir, 'config', 'cameras', 'cameras.yaml')
    with open(cameras_path) as f:
        cameras = yaml.safe_load(f)
    cam = cameras['realsense_d555']

    yolo_config_path = os.path.join(pkg_dir, 'config', 'yolo', 'yolo_obb.yaml')
    with open(yolo_config_path) as f:
        yolo_cfg = yaml.safe_load(f)

    calib_file = os.path.join(
        pkg_dir, 'config', 'cameras', 'color_realsense_d555_calib.yaml'
    )
    models_dir = os.path.join(pkg_dir, 'models')
    model_file_path = os.path.join(models_dir, str(yolo_cfg['model_file_path']))
    engine_file_path = os.path.join(models_dir, str(yolo_cfg['engine_file_path']))

    input_image_width = int(cam['image_width'])
    input_image_height = int(cam['image_height'])

    # RealSense Camera
    realsense_node = ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        name='camera',
        namespace='camera',
        parameters=[
            {
                'enable_color': True,
                'rgb_camera.color_profile': '896,504,15',
                'rgb_camera.color_format': 'RGB8',
                'rgb_camera.enable_auto_exposure': True,
                'enable_depth': True,
                'depth_module.depth_profile': '896,504,15',
                'depth_module.depth_format': 'Z16',
                'depth_module.enable_auto_exposure': True,
                'depth_module.emitter_enabled': False,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_gyro': False,
                'enable_accel': False,
                'enable_motion': False,
                'publish_tf': False,
                'enable_sync': False,
            }
        ],
        remappings=[
            # Color image is NOT remapped here — image_undistort reads the raw
            # topic and publishes to cam["image_topic"] (image_rect).
            (cam['raw_depth_image_topic'], cam['depth_image_topic']),
            (cam['raw_depth_camera_info_topic'], cam['depth_camera_info_topic']),
        ],
    )

    # Undistorts the color image using the calibration YAML.
    # In undistort mode  : loads K+D from calib file at startup, publishes
    #                      rectified image + zero-distortion camera_info.
    # In passthrough mode: relays raw image and driver camera_info as-is.
    image_undistort_node = ComposableNode(
        package='perception_setup',
        plugin='perception_setup::ImageUndistort',
        name='color_image_undistort',
        parameters=[
            {
                'image_topic': cam['raw_color_image_topic'],
                'camera_info_file': calib_file,
                'raw_camera_info_topic': cam['raw_color_camera_info_topic'],
                'output_image_topic': cam['image_topic'],
                'output_camera_info_topic': cam['camera_info_topic'],
                'enable_undistort': LaunchConfiguration('enable_undistort'),
                'image_qos': 'reliable', # Isaac ros only works with reliable QoS
            }
        ],
    )

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
            ('image_raw', cam['image_topic']),
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
            realsense_node,
            image_undistort_node,
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
            'camera_info_input_topic': cam['camera_info_topic'],
            'tensor_output_topic': TENSOR_OUTPUT_TOPIC,
        }.items(),
    )

    yolo_obb_visualizer = Node(
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
    ) if bool(yolo_cfg['enable_visualizer']) else None

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
                        'depth_image_sub_topic': cam['depth_image_topic'],
                        'detections_sub_topic': yolo_cfg['detection_topic'],
                        'depth_image_info_topic': cam['depth_camera_info_topic'],
                        'depth_frame_id': 'front_camera_depth_optical',
                        'color_frame_id': 'front_camera_color_optical',
                        'landmarks_pub_topic': '/valve_landmarks',
                        'output_frame_id': 'front_camera_depth_optical',
                        'drone': LaunchConfiguration('drone'),
                        'undistort_detections': LaunchConfiguration('undistort_detections'),
                        'debug_visualize': LaunchConfiguration('debug_visualize'),
                    },
                ],
            )
        ],
        output='screen',
    )

    # Collect all actions
    actions = [
        tensor_rt_container,
        dnn_image_encoder_launch,
        valve_detection_container,
    ]

    if yolo_obb_visualizer:
        actions.append(yolo_obb_visualizer)

    return actions


def generate_launch_description():
    """Generate the launch description."""
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                'enable_undistort',
                default_value='true',
                description='Undistort color image before passing to YOLO',
            ),
            # Valve Detection Options
            DeclareLaunchArgument(
                'drone',
                default_value='nautilus',
                description='Robot name, prepended to TF frame IDs',
            ),
            DeclareLaunchArgument(
                'undistort_detections',
                default_value='false', # If this is enabled then enable_undistort must be disabled to avoid double undistortion. TODO: I had problems getting this working, work around is to just use enable_undistort
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
