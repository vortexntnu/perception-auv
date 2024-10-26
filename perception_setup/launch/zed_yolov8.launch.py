import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import UnlessCondition, IfCondition

# Define the default container name for composable nodes
DEFAULT_CONTAINER_NAME = 'zed_yolov8_container'

def generate_launch_description():
    # Declare common launch arguments
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value=DEFAULT_CONTAINER_NAME,
        description='Name of the container to load the composable nodes into'
    )
    run_standalone_arg = DeclareLaunchArgument(
        'run_standalone',
        default_value='False',
        description='Run the nodes as standalone if set to True'
    )

    # Launch configuration for the container name and run_standalone
    container_name = LaunchConfiguration('container_name')
    run_standalone = LaunchConfiguration('run_standalone')

    # Path to the ZED launch file (adjust the path as needed)
    zed_launch_path = os.path.join(get_package_share_directory('perception_setup'), 'launch', 'zed_composable_node_launch.py')

    # Include the ZED launch file
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([zed_launch_path]),
        launch_arguments={
            'container_name': container_name,
            'run_standalone': run_standalone, # Pass the run_standalone argument to the ZED launch
        }.items(),
    )

    # Define the default values for the YOLOv8 composable node configurations
    model_file_path_arg = DeclareLaunchArgument(
        'model_file_path',
        default_value=os.path.join(get_package_share_directory('perception_setup'), 'resources', 'yolov4-tiny-3l-416-416.onnx'),
        description='Path to the ONNX model file'
    )
    engine_file_path_arg = DeclareLaunchArgument(
        'engine_file_path',
        default_value=os.path.join(get_package_share_directory('perception_setup'), 'resources', 'yolov4-tiny-3l-416-416.trt'),
        description='Path to the TensorRT engine file'
    )
  
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Confidence threshold for object detection'
    )
    nms_threshold_arg = DeclareLaunchArgument(
        'nms_threshold',
        default_value='0.5',
        description='Non-maximum suppression threshold for object detection'
    )

    image_input_topic_arg = DeclareLaunchArgument(
        'image_input_topic',
        default_value='/image_rect',
        description='Input image topic for object detection'
    )
    camera_info_input_topic_arg = DeclareLaunchArgument(
        'camera_info_input_topic',
        default_value='/camera_info_rect',
        description='Input camera info topic for object detection'
    )
    tensor_output_topic_arg = DeclareLaunchArgument(
        'tensor_output_topic',
        default_value='/tensor_pub',
        description='Output tensor topic for object detection'
    )

    # TensorRT and YOLOv8 composable node configurations
    model_file_path = LaunchConfiguration('model_file_path')
    engine_file_path = LaunchConfiguration('engine_file_path')

    confidence_threshold = LaunchConfiguration('confidence_threshold')
    nms_threshold = LaunchConfiguration('nms_threshold')

    image_input_topic = LaunchConfiguration('image_input_topic')
    camera_info_input_topic = LaunchConfiguration('camera_info_input_topic')
    tensor_output_topic = LaunchConfiguration('tensor_output_topic')

    tensor_rt_node = ComposableNode(
        name='tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[{
            
            'model_file_path': model_file_path,
            'engine_file_path': engine_file_path,
            'output_binding_names': '["output0"]',
            'output_tensor_names': '["output_tensor"]',
            'input_tensor_names': '["input_tensor"]',
            'input_binding_names': '["images"]',
            'verbose': 'False',
            'force_engine_update': 'False'
        }]
    )

    yolov8_decoder_node = ComposableNode(
        name='yolov8_decoder_node',
        package='isaac_ros_yolov8',
        plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        parameters=[{
            'confidence_threshold': confidence_threshold,
            'nms_threshold': nms_threshold,
        }],
        remappings=[('tensor', tensor_output_topic),
                    ('detections_output', '/yolov8/detections')],
    )

    # Include the DNN Image Encoder launch (adjust the paths accordingly)
    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')
    yolov8_encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')]
        ),
        launch_arguments={
            'input_image_width': '640',  # Adjust as needed
            'input_image_height': '640',
            'network_image_width': '640',
            'network_image_height': '640',
            'image_mean': '[0.0, 0.0, 0.0]',
            'image_stddev': '[1.0, 1.0, 1.0]',
            'attach_to_shared_component_container': 'True',
            'component_container_name': container_name,
            'dnn_image_encoder_namespace': 'yolov8_encoder',
            'image_input_topic': image_input_topic,
            'camera_info_input_topic': camera_info_input_topic,
            'tensor_output_topic': tensor_output_topic,
        }.items(),
    )

    # Define the image resize node for the left camera to visualize the YOLOv8 output
    image_resize_node_left = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        name='image_resize_node_left',
        parameters=[{
                'output_width': '640',
                'output_height': '640',
                'encoding_desired': 'rgb8',
        }],
        remappings=[
            ('camera_info', 'front_stereo_camera/left_rgb/camerainfo'),
            ('image', 'front_stereo_camera/left_rgb/image_raw'),
            ('resize/camera_info', 'front_stereo_camera/left_rgb/camerainfo_resize'),
            ('resize/image', 'front_stereo_camera/left_rgb/image_resize')]
    )


    yolov8_visualizer_node = Node(
        package='isaac_ros_yolov8',
        executable='isaac_ros_yolov8_visualizer.py',
        name='yolov8_visualizer',
        remappings=[('yolov8_encoder/resize/image', 'front_stereo_camera/left_rgb/image_resize'),
                    ('detections_output', '/yolov8/detections')],

    )

    # Combine the ZED launch and the YOLOv8 setup into a single launch description
    return LaunchDescription([
        container_name_arg,
        run_standalone_arg,
        model_file_path_arg,
        engine_file_path_arg,
        confidence_threshold_arg,
        nms_threshold_arg,
        image_input_topic_arg,
        camera_info_input_topic_arg,
        tensor_output_topic_arg,

        zed_launch,
        yolov8_encoder_launch,
        ComposableNodeContainer(
            name=container_name,
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[tensor_rt_node, yolov8_decoder_node, image_resize_node_left],
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO'],
            condition=IfCondition(run_standalone),
        ),
        yolov8_visualizer_node,
    ])
