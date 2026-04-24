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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('perception_setup')
    models_dir = os.path.join(pkg_dir, 'models')
    drone = LaunchConfiguration('drone').perform(context)

    with open(os.path.join(pkg_dir, 'config', 'yolo', 'yolo_obb.yaml')) as f:
        yolo_cfg = yaml.safe_load(f)

    calib_file = os.path.join(
        pkg_dir, 'config', 'cameras', 'color_realsense_d555_calib.yaml'
    )
    model_file_path = os.path.join(models_dir, 'obb_best.onnx')
    engine_file_path = os.path.join(models_dir, 'obb_best.engine')

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
            (
                '/camera/camera/depth/image_rect_raw',
                f'/{drone}/depth_camera/image_depth',
            ),
            ('/camera/camera/depth/camera_info', f'/{drone}/depth_camera/camera_info'),
        ],
    )

    image_undistort_node = ComposableNode(
        package='perception_setup',
        plugin='perception_setup::ImageUndistort',
        name='color_image_undistort',
        parameters=[
            {
                'image_topic': '/camera/camera/color/image_raw',
                'camera_info_file': calib_file,
                'raw_camera_info_topic': '/camera/camera/color/camera_info',
                'output_image_topic': f'/{drone}/front_camera/image_color',
                'output_camera_info_topic': f'/{drone}/front_camera/camera_info',
                'enable_undistort': LaunchConfiguration('enable_undistort'),
                'image_qos': 'reliable',  # Isaac ros only works with reliable QoS
            }
        ],
    )

    image_format_converter = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_converter',
        parameters=[
            {
                'encoding_desired': 'rgb8',
                'image_width': 896,
                'image_height': 504,
                'input_qos': 'sensor_data',
            }
        ],
        remappings=[
            ('image_raw', f'/{drone}/front_camera/image_color'),
            ('image', '/yolo_obb/internal/converted_image'),
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
                'verbose': True,
                'force_engine_update': False,
                'tensor_output_topic': '/yolo_obb/tensor_pub',
            }
        ],
        remappings=[
            ('tensor_pub', '/yolo_obb/tensor_pub'),
            ('tensor_sub', '/yolo_obb/tensor_sub'),
        ],
    )

    yolo_obb_decoder_node = ComposableNode(
        name='yolo_obb_decoder_node',
        package='isaac_ros_yolov26_obb',
        plugin='nvidia::isaac_ros::yolov26_obb::YoloV26OBBDecoderNode',
        parameters=[
            {
                'tensor_input_topic': '/yolo_obb/tensor_sub',
                'confidence_threshold': float(yolo_cfg['confidence_threshold']),
                'num_detections': int(yolo_cfg['num_detections']),
                'detections_topic': '/obb_detections_output',
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
            'input_image_width': str(896),
            'input_image_height': str(504),
            'network_image_width': str(yolo_cfg['network_image_width']),
            'network_image_height': str(yolo_cfg['network_image_height']),
            'image_mean': str(yolo_cfg['image_mean']),
            'image_stddev': str(yolo_cfg['image_stddev']),
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'obb_tensor_rt_container',
            'dnn_image_encoder_namespace': 'yolo_obb_encoder/internal',
            'image_input_topic': '/yolo_obb/internal/converted_image',
            'camera_info_input_topic': f'/{drone}/front_camera/camera_info',
            'tensor_output_topic': '/yolo_obb/tensor_pub',
        }.items(),
    )

    yolo_obb_visualizer = Node(
        package='isaac_ros_yolov26_obb',
        executable='isaac_ros_yolov26_obb_visualizer.py',
        name='yolo_obb_visualizer',
        parameters=[
            {
                'detections_topic': '/obb_detections_output',
                'image_topic': '/yolo_obb_encoder/internal/resize/image',
                'output_image_topic': '/yolo_obb_processed_image',
                'class_names_yaml': str({0: 'valve'}),
            }
        ],
    )

    valve_detection_tuning = os.path.join(
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
                    valve_detection_tuning,
                    {
                        'depth_image_sub_topic': f'/{drone}/depth_camera/image_depth',
                        'detections_sub_topic': '/obb_detections_output',
                        'depth_image_info_topic': f'/{drone}/depth_camera/camera_info',
                        'depth_frame_id': 'front_camera_depth_optical',
                        'color_frame_id': 'front_camera_color_optical',
                        'landmarks_pub_topic': '/valve_landmarks',
                        'output_frame_id': 'front_camera_depth_optical',
                        'drone': LaunchConfiguration('drone'),
                        'undistort_detections': LaunchConfiguration(
                            'undistort_detections'
                        ),
                        'debug_visualize': LaunchConfiguration('debug_visualize'),
                        'clamp_rotation': LaunchConfiguration('clamp_rotation'),
                        'use_hardcoded_extrinsic': LaunchConfiguration(
                            'use_hardcoded_extrinsic'
                        ),
                        'extrinsic_tx': LaunchConfiguration('extrinsic_tx'),
                        'extrinsic_ty': LaunchConfiguration('extrinsic_ty'),
                        'extrinsic_tz': LaunchConfiguration('extrinsic_tz'),
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
                'input_topic': '/yolo_obb_processed_image',
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

    actions = [
        tensor_rt_container,
        dnn_image_encoder_launch,
        valve_detection_container,
        image_to_gstreamer_node,
    ]

    actions.append(yolo_obb_visualizer)

    return actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                'enable_undistort',
                default_value='true',
                description='Undistort color image before passing to YOLO',
            ),
            DeclareLaunchArgument(
                'drone',
                default_value='nautilus',
                description='Robot name, prepended to TF frame IDs',
            ),
            DeclareLaunchArgument(
                'undistort_detections',
                # If enabled, disable enable_undistort to avoid double-undistortion.
                default_value='false',
                description='Undistort detections using color camera distortion',
            ),
            DeclareLaunchArgument(
                'debug_visualize',
                default_value='true',
                description='Enable debug visualization topics',
            ),
            DeclareLaunchArgument(
                'clamp_rotation',
                default_value='true',
                description='Clamp valve handle angle to 0-90 deg (0=vertical, 90=horizontal)',
            ),
            DeclareLaunchArgument(
                'use_hardcoded_extrinsic',
                default_value='true',
                description='Use hardcoded depth-to-color extrinsic instead of TF lookup',
            ),
            DeclareLaunchArgument(
                'extrinsic_tx',
                default_value='-0.059',
                description='Hardcoded extrinsic translation X (metres)',
            ),
            DeclareLaunchArgument(
                'extrinsic_ty',
                default_value='0.0',
                description='Hardcoded extrinsic translation Y (metres)',
            ),
            DeclareLaunchArgument(
                'extrinsic_tz',
                default_value='0.0',
                description='Hardcoded extrinsic translation Z (metres)',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
