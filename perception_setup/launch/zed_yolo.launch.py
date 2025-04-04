import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# Define the default container name for composable nodes
DEFAULT_CONTAINER_NAME = 'zed_yolov8_container'


def generate_launch_description():
    # Declare common launch arguments
    config_file_common = os.path.join(
        get_package_share_directory('perception_setup'),
        'config',
        'zed_driver_params.yaml',
    )

    # URDF/xacro file to be loaded by the Robot State Publisher node
    xacro_path = os.path.join(
        get_package_share_directory('zed_wrapper'), 'urdf', 'zed_descr.urdf.xacro'
    )

    # Define the default values for the YOLOv8 composable node configurations

    image_format_converter_node_left = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_node_left',
        parameters=[
            {
                'encoding_desired': 'rgb8',
                'image_width': 1280,
                'image_height': 720,
            }
        ],
        remappings=[
            ('image_raw', 'zed_node/left/image_rect_color'),
            ('image', 'image_rect'),
        ],
    )
    zed_wrapper_component = ComposableNode(
        package='zed_components',
        name='zed_node',
        plugin='stereolabs::ZedCamera',
        parameters=[
            config_file_common,  # Common parameters
        ],
        remappings=[
            ('zed_node/left/camera_info', '/camera_info_rect'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    valve_detection_node = ComposableNode(
        package='valve_detection',
        plugin='ValveDetectionNode',
        name='valve_detection_node',
        parameters=[
            os.path.join(
                get_package_share_directory('valve_detection'),
                'config',
                'valve_detection_params.yaml',
            )
        ],
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='zed_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': Command(
                    [
                        'xacro',
                        ' ',
                        str(xacro_path),
                        ' ',
                        'camera_name:=zed',
                        ' ',
                        'camera_model:=zed2i',
                    ]
                )
            }
        ],
    )

    # TensorRT parameters
    model_file_path = LaunchConfiguration('model_file_path')
    engine_file_path = LaunchConfiguration('engine_file_path')
    input_tensor_names = LaunchConfiguration('input_tensor_names')
    input_binding_names = LaunchConfiguration('input_binding_names')
    output_tensor_names = LaunchConfiguration('output_tensor_names')
    output_binding_names = LaunchConfiguration('output_binding_names')
    verbose = LaunchConfiguration('verbose')
    force_engine_update = LaunchConfiguration('force_engine_update')

    # YOLOv8 Decoder parameters
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    nms_threshold = LaunchConfiguration('nms_threshold')

    tensor_rt_node = ComposableNode(
        name='tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[
            {
                'model_file_path': model_file_path,
                'engine_file_path': engine_file_path,
                'output_binding_names': output_binding_names,
                'output_tensor_names': output_tensor_names,
                'input_tensor_names': input_tensor_names,
                'input_binding_names': input_binding_names,
                'verbose': verbose,
                'force_engine_update': force_engine_update,
            }
        ],
    )
    yolov8_decoder_node = ComposableNode(
        name='yolov8_decoder_node',
        package='isaac_ros_yolov8',
        plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        parameters=[
            {
                'confidence_threshold': confidence_threshold,
                'nms_threshold': nms_threshold,
                'num_classes': 1,
            }
        ],
    )

    network_image_width = LaunchConfiguration('network_image_width')
    network_image_height = LaunchConfiguration('network_image_height')
    input_encoding = LaunchConfiguration('input_encoding')
    image_mean = LaunchConfiguration('image_mean')
    image_stddev = LaunchConfiguration('image_stddev')
    image_input_topic = LaunchConfiguration('image_input_topic')
    camera_info_input_topic = LaunchConfiguration('camera_info_input_topic')

    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')

    network_image_width = DeclareLaunchArgument(
        'network_image_width',
        default_value='640',
        description='The input image width that the network expects',
    )
    network_image_height = DeclareLaunchArgument(
        'network_image_height',
        default_value='640',
        description='The input image height that the network expects',
    )
    input_encoding = DeclareLaunchArgument(
        'input_encoding',
        default_value='rgb8',
        description='The desired image format encoding',
    )
    image_mean = DeclareLaunchArgument(
        'image_mean',
        default_value='[0.0, 0.0, 0.0]',
        description='The mean for image normalization',
    )
    image_stddev = DeclareLaunchArgument(
        'image_stddev',
        default_value='[1.0, 1.0, 1.0]',
        description='The standard deviation for image normalization',
    )
    model_file_path = DeclareLaunchArgument(
        'model_file_path',
        default_value=os.path.join(
            get_package_share_directory('perception_setup'), 'models', 'best200.onnx'
        ),
        description='Path to the ONNX model file',
    )
    engine_file_path = DeclareLaunchArgument(
        'engine_file_path',
        default_value=os.path.join(
            get_package_share_directory('perception_setup'),
            'models',
            'yolo-04-04.engine',
        ),
        description='Path to the TensorRT engine file',
    )
    input_tensor_names = DeclareLaunchArgument(
        'input_tensor_names',
        default_value='["input_tensor"]',
        description='A list of tensor names to bound to the specified input binding names',
    )
    input_binding_names = DeclareLaunchArgument(
        'input_binding_names',
        default_value='["images"]',
        description='A list of input tensor binding names (specified by model)',
    )
    image_input_topic = DeclareLaunchArgument(
        'image_input_topic',
        default_value='/image_rect',
        description='Input image topic name',
    )
    camera_info_input_topic = DeclareLaunchArgument(
        'camera_info_input_topic',
        default_value='/camera_info_rect',
        description='Input camera info topic name',
    )
    output_tensor_names = DeclareLaunchArgument(
        'output_tensor_names',
        default_value='["output_tensor"]',
        description='A list of tensor names to bound to the specified output binding names',
    )
    output_binding_names = DeclareLaunchArgument(
        'output_binding_names',
        default_value='["output0"]',
        description='A list of output tensor binding names (specified by model)',
    )
    verbose = DeclareLaunchArgument(
        'verbose',
        default_value='False',
        description='Whether TensorRT should verbosely log or not',
    )
    force_engine_update = DeclareLaunchArgument(
        'force_engine_update',
        default_value='False',
        description='Whether TensorRT should update the TensorRT engine file or not',
    )
    confidence_threshold = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.25',
        description='Confidence threshold to filter candidate detections during NMS',
    )
    nms_threshold = DeclareLaunchArgument(
        'nms_threshold', default_value='0.45', description='NMS IOU threshold'
    )

    yolov8_encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')]
        ),
        launch_arguments={
            'input_image_width': '1280',
            'input_image_height': '720',
            'network_image_width': LaunchConfiguration('network_image_width'),
            'network_image_height': LaunchConfiguration('network_image_height'),
            'image_mean': LaunchConfiguration('image_mean'),
            'image_stddev': LaunchConfiguration('image_stddev'),
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'yolo_container',
            'dnn_image_encoder_namespace': 'yolov8_encoder',
            'image_input_topic': LaunchConfiguration('image_input_topic'),
            'camera_info_input_topic': LaunchConfiguration('camera_info_input_topic'),
            'tensor_output_topic': '/tensor_pub',
            'input_encoding': LaunchConfiguration('input_encoding'),
        }.items(),
    )

    yolov8_visualizer_node = Node(
        package='isaac_ros_yolov8',
        executable='isaac_ros_yolov8_visualizer.py',
        name='yolov8_visualizer',
    )

    return LaunchDescription(
        [
            network_image_width,
            network_image_height,
            input_encoding,
            image_mean,
            image_stddev,
            model_file_path,
            engine_file_path,
            input_tensor_names,
            input_binding_names,
            image_input_topic,
            camera_info_input_topic,
            output_tensor_names,
            output_binding_names,
            verbose,
            force_engine_update,
            confidence_threshold,
            nms_threshold,
            rsp_node,
            yolov8_encoder_launch,
            # yolov8_visualizer_node,
            ComposableNodeContainer(
                name='yolo_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[
                    tensor_rt_node,
                    yolov8_decoder_node,
                    image_format_converter_node_left,
                    zed_wrapper_component,
                    valve_detection_node,
                ],
                output='screen',
                arguments=['--ros-args', '--log-level', 'INFO'],
            ),
            # enable_valve_detection_arg,
            # yolov8_visualizer_node,
        ]
    )
