"""Blackfly S -> YOLO Segmentation -> YOLO Classification -> GStreamer stream.

Pipeline:
1. Blackfly S camera driver publishes raw BGR image
2. YOLO segmentation detects pipeline instances and produces a segmentation mask
   and visualization image
3. YOLO classification runs on the segmentation mask to classify the pipeline state
4. image_to_gstreamer streams the segmentation visualization image over RTP/H.265
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('perception_setup')
    models_dir = os.path.join(pkg_dir, 'models')
    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')
    drone = LaunchConfiguration('drone').perform(context)

    with open(os.path.join(pkg_dir, 'config', 'yolo', 'yolo_seg.yaml')) as f:
        seg_cfg = yaml.safe_load(f)
    with open(os.path.join(pkg_dir, 'config', 'yolo', 'yolo_cls.yaml')) as f:
        cls_cfg = yaml.safe_load(f)

    # -------------------------------------------------------------------------
    # Camera
    # -------------------------------------------------------------------------
    spinnaker_map = os.path.join(pkg_dir, 'config', 'cameras', 'blackfly_s_params.yaml')
    driver_params = os.path.join(
        pkg_dir, 'config', 'cameras', 'blackfly_s_driver_params.yaml'
    )
    calib_path = os.path.join(pkg_dir, 'config', 'cameras', 'blackfly_s_calib.yaml')

    camera_container = ComposableNodeContainer(
        name='down_cam_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='spinnaker_camera_driver',
                plugin='spinnaker_camera_driver::CameraDriver',
                name='blackfly_s',
                parameters=[
                    driver_params,
                    {
                        'parameter_file': spinnaker_map,
                        'serial_number': '23494258',
                        'camerainfo_url': f'file://{calib_path}',
                        'reliable_qos': True,
                    },
                ],
                remappings=[
                    ('~/control', '/exposure_control/control'),
                    ('/blackfly_s/image_raw',   f'/{drone}/down_camera/image_color'),
                    ('/blackfly_s/camera_info', f'/{drone}/down_camera/camera_info'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(LaunchConfiguration('enable_camera')),
            ),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # -------------------------------------------------------------------------
    # YOLO Segmentation
    # -------------------------------------------------------------------------
    seg_image_format_converter = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='seg_image_format_converter',
        parameters=[
            {
                'encoding_desired': 'rgb8',
                'image_width': 720,
                'image_height': 540,
                'input_qos': 'sensor_data',
            }
        ],
        remappings=[
            ('image_raw', f'/{drone}/down_camera/image_color'),
            ('image', '/down_cam/yolo_seg/internal/converted_image'),
        ],
    )

    seg_tensor_rt_node = ComposableNode(
        name='seg_tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[
            {
                'model_file_path': os.path.join(models_dir, 'seg_pipe_real.onnx'),
                'engine_file_path': os.path.join(models_dir, 'seg_pipe_real.engine'),
                'output_binding_names': seg_cfg['output_binding_names'],
                'output_tensor_names': seg_cfg['output_tensor_names'],
                'input_tensor_names': seg_cfg['input_tensor_names'],
                'input_binding_names': seg_cfg['input_binding_names'],
                'verbose': True,
                'force_engine_update': False,
                'tensor_output_topic': '/down_cam/yolo_seg/tensor_pub',
            }
        ],
        remappings=[
            ('tensor_pub', '/down_cam/yolo_seg/tensor_pub'),
            ('tensor_sub', '/down_cam/yolo_seg/tensor_sub'),
        ],
    )

    seg_decoder_node = ComposableNode(
        name='seg_decoder_node',
        package='isaac_ros_yolov26_seg',
        plugin='nvidia::isaac_ros::yolov26_seg::YoloV26SegDecoderNode',
        parameters=[
            {
                'tensor_input_topic': '/down_cam/yolo_seg/tensor_sub',
                'confidence_threshold': float(seg_cfg['confidence_threshold']),
                'num_detections': int(seg_cfg['num_detections']),
                'mask_width': int(seg_cfg['mask_width']),
                'mask_height': int(seg_cfg['mask_height']),
                'num_protos': int(seg_cfg['num_protos']),
                'network_image_width': int(seg_cfg['network_image_width']),
                'network_image_height': int(seg_cfg['network_image_height']),
                'output_mask_width': 640,
                'output_mask_height': 640,
                'detections_topic': '/down_cam/seg_detections_output',
                'mask_topic': '/down_cam/yolo_seg_mask',
            }
        ],
    )

    seg_container = ComposableNodeContainer(
        name='down_cam_seg_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            seg_image_format_converter,
            seg_tensor_rt_node,
            seg_decoder_node,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    seg_encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')
        ),
        launch_arguments={
            'input_image_width': str(720),
            'input_image_height': str(540),
            'network_image_width': str(seg_cfg['network_image_width']),
            'network_image_height': str(seg_cfg['network_image_height']),
            'image_mean': str(seg_cfg['image_mean']),
            'image_stddev': str(seg_cfg['image_stddev']),
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'down_cam_seg_container',
            'dnn_image_encoder_namespace': 'down_cam/yolo_seg_encoder/internal',
            'image_input_topic': '/down_cam/yolo_seg/internal/converted_image',
            'camera_info_input_topic': f'/{drone}/down_camera/camera_info',
            'tensor_output_topic': '/down_cam/yolo_seg/tensor_pub',
        }.items(),
    )

    seg_visualizer = Node(
        package='isaac_ros_yolov26_seg',
        executable='isaac_ros_yolov26_seg_visualizer.py',
        name='seg_visualizer',
        parameters=[
            {
                'detections_topic': '/down_cam/seg_detections_output',
                'image_topic': '/down_cam/yolo_seg_encoder/internal/resize/image',
                'mask_topic': '/down_cam/yolo_seg_mask',
                'output_image_topic': '/down_cam/yolo_seg_processed_image',
                'class_names_yaml': str({0: 'pipeline'}),
            }
        ],
    )

    # -------------------------------------------------------------------------
    # YOLO Classification
    # -------------------------------------------------------------------------
    cls_image_format_converter = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='cls_image_format_converter',
        parameters=[
            {
                'encoding_desired': 'rgb8',
                'image_width': 640,
                'image_height': 640,
                'input_qos': 'sensor_data',
            }
        ],
        remappings=[
            ('image_raw', '/down_cam/yolo_seg_mask'),
            ('image', '/down_cam/yolo_cls/internal/converted_image'),
        ],
    )

    cls_tensor_rt_node = ComposableNode(
        name='cls_tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[
            {
                'model_file_path': os.path.join(models_dir, 'end-of-pipeline-yolov26.onnx'),
                'engine_file_path': os.path.join(models_dir, 'end-of-pipeline-yolov26.engine'),
                'output_binding_names': cls_cfg['output_binding_names'],
                'output_tensor_names': cls_cfg['output_tensor_names'],
                'input_tensor_names': cls_cfg['input_tensor_names'],
                'input_binding_names': cls_cfg['input_binding_names'],
                'verbose': True,
                'force_engine_update': False,
                'tensor_output_topic': '/down_cam/yolo_cls/tensor_pub',
            }
        ],
        remappings=[
            ('tensor_pub', '/down_cam/yolo_cls/tensor_pub'),
            ('tensor_sub', '/down_cam/yolo_cls/tensor_sub'),
        ],
    )

    cls_decoder_node = ComposableNode(
        name='cls_decoder_node',
        package='isaac_ros_yolov26_cls',
        plugin='nvidia::isaac_ros::yolov26_cls::YoloV26ClsDecoderNode',
        parameters=[
            {
                'tensor_input_topic': '/down_cam/yolo_cls/tensor_sub',
                'confidence_threshold': float(cls_cfg['confidence_threshold']),
                'num_classes': int(cls_cfg['num_classes']),
                'class_topic': '/down_cam/classification_output',
            }
        ],
    )

    cls_container = ComposableNodeContainer(
        name='down_cam_cls_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            cls_image_format_converter,
            cls_tensor_rt_node,
            cls_decoder_node,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    cls_encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')
        ),
        launch_arguments={
            'input_image_width': str(640),
            'input_image_height': str(640),
            'network_image_width': str(cls_cfg['network_image_width']),
            'network_image_height': str(cls_cfg['network_image_height']),
            'image_mean': str(cls_cfg['image_mean']),
            'image_stddev': str(cls_cfg['image_stddev']),
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'down_cam_cls_container',
            'dnn_image_encoder_namespace': 'down_cam/yolo_cls_encoder/internal',
            'image_input_topic': '/down_cam/yolo_cls/internal/converted_image',
            'camera_info_input_topic': '/camera/camera/color/camera_info_rect',
            'tensor_output_topic': '/down_cam/yolo_cls/tensor_pub',
        }.items(),
    )

    cls_visualizer = Node(
        package='isaac_ros_yolov26_cls',
        executable='isaac_ros_yolov26_cls_visualizer.py',
        name='cls_visualizer',
        parameters=[
            {
                'class_topic': '/down_cam/classification_output',
                'image_topic': '/down_cam/yolo_cls_encoder/internal/resize/image',
                'output_image_topic': '/down_cam/yolo_cls_processed_image',
                'class_names_yaml': str({0: 'continue', 1: 'end'}),
            }
        ],
    )

    # -------------------------------------------------------------------------
    # GStreamer — streams the segmentation visualization (bgr8)
    # -------------------------------------------------------------------------
    image_to_gstreamer_node = Node(
        package='image_to_gstreamer',
        executable='image_to_gstreamer_node',
        name='image_to_gstreamer_node',
        additional_env={'EGL_PLATFORM': 'surfaceless'},
        parameters=[
            {
                'input_topic': '/down_cam/yolo_seg_processed_image',
                'host': '10.0.0.68',
                'port': 5001,
                'bitrate': 500000,
                'framerate': 15,
                'preset_level': 1,
                'iframe_interval': 15,
                'control_rate': 1,
                'pt': 96,
                'config_interval': 1,
                'format': 'BGR',
            }
        ],
        output='screen',
    )

    actions = [
        camera_container,
        seg_container,
        seg_encoder_launch,
        cls_container,
        cls_encoder_launch,
        image_to_gstreamer_node,
    ]

    actions.append(seg_visualizer)
    actions.append(cls_visualizer)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'drone',
                default_value='nautilus',
                description='Drone name, prepended to all published topics (e.g. /nautilus/down_camera/image_color)',
            ),
            DeclareLaunchArgument(
                'enable_camera',
                default_value='true',
                description='Enable FLIR Blackfly S camera component',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
