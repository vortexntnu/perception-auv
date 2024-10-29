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
        default_value=os.path.join(get_package_share_directory('perception_setup'), 'models', 'best_red.onnx'),
        description='Path to the ONNX model file'
    )
    engine_file_path_arg = DeclareLaunchArgument(
        'engine_file_path',
        default_value=os.path.join(get_package_share_directory('perception_setup'), 'models', 'best_red.onnx.engine'),
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
        default_value='image_rect',
        description='Input image topic for object detection'
    )
    camera_info_input_topic_arg = DeclareLaunchArgument(
        'camera_info_input_topic',
        default_value='/zed/zed_node/left/camera_info',
        description='Input camera info topic for object detection'
    )
    tensor_output_topic_arg = DeclareLaunchArgument(
        'tensor_output_topic',
        default_value='/yolov8_encoder/image_tensor_out',
        description='Output tensor topic for object detection'
    )

    input_tensor_name_arg = DeclareLaunchArgument(
            'input_tensor_names',
            default_value='["input_tensor"]',
            description='A list of tensor names to bound to the specified input binding names')
    
    output_tensor_name_arg = DeclareLaunchArgument(
            'output_tensor_names',
            default_value='["output_tensor"]',
            description='A list of tensor names to bound to the specified output binding names')

    # TensorRT and YOLOv8 composable node configurations
    model_file_path = LaunchConfiguration('model_file_path')
    engine_file_path = LaunchConfiguration('engine_file_path')

    confidence_threshold = LaunchConfiguration('confidence_threshold')
    nms_threshold = LaunchConfiguration('nms_threshold')

    image_input_topic = LaunchConfiguration('image_input_topic')
    camera_info_input_topic = LaunchConfiguration('camera_info_input_topic')
    tensor_output_topic = LaunchConfiguration('tensor_output_topic')

    input_tensor_names = LaunchConfiguration('input_tensor_names')
    output_tensor_names = LaunchConfiguration('output_tensor_names')
    

    # First step is to convert from bgr8 to rgb8
    image_format_converter_node_left = ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                name='image_format_node_left',
                parameters=[{
                    'encoding_desired': 'rgb8',
                    'image_width': 1280,
                    'image_height': 720,
                }],
                remappings=[
                    ('image_raw', '/zed/zed_node/left/image_rect_color'),
                    ('image', '/yolov8_encoder/image_rect')]
            )

    # To find binding names run 
    # trtexec --loadEngine=<path_to_your_model.engine> --verbose
    tensor_rt_node = ComposableNode(
        name='tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[{
            
            'model_file_path': model_file_path,
            'engine_file_path': engine_file_path,
            'output_binding_names': ["output0"],
            'output_tensor_names': ["output_tensor"],  # ROS-specific output tensor name
            'input_tensor_names': ["input_tensor"],    # ROS-specific input tensor name
            'input_binding_names': ["images"],         # TensorRT engine binding names
            'verbose': False,
            'force_engine_update': False
        }],
        remappings=[('tensor_pub', '/yolov8_encoder/image_tensor'),
                    ('tensor_sub', '/yolov8_encoder/image_tensor_out')],
    )

    yolov8_decoder_node = ComposableNode(
        name='yolov8_decoder_node',
        package='isaac_ros_yolov8',
        plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        parameters=[{
            'confidence_threshold': confidence_threshold,
            'nms_threshold': nms_threshold,
        }],
        remappings=[
            ('tensor_sub', '/yolov8_encoder/image_tensor_out'),
                    ('detections_output', '/yolov8/detections')],
    )

    # Include the DNN Image Encoder launch (adjust the paths accordingly)
    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')
    yolov8_encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')]
        ),
        launch_arguments={
            'input_image_width': '1280',
            'input_image_height': '720',
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
    # image_resize_node_left = ComposableNode(
    #     package='isaac_ros_image_proc',
    #     plugin='nvidia::isaac_ros::image_proc::ResizeNode',
    #     name='image_resize_node_left',
    #     parameters=[{
    #             'output_width': 640,
    #             'output_height': 640,
    #             'encoding_desired': 'rgb8',
    #     }],
    #     remappings=[
    #         ('camera_info', '/zed/zed_node/left/camera_info'),
    #         ('image', '/zed/zed_node/left/image_rect_color'),
    #         ('resize/camera_info', '/zed/zed_node/left/camera_info_resize'),
    #         ('resize/image', '/zed/zed_node/left/image_rect_color')]
    # )


    # yolov8_visualizer_node = Node(
    #     package='isaac_ros_yolov8',
    #     executable='isaac_ros_yolov8_visualizer.py',
    #     name='yolov8_visualizer',
    #     remappings=[('yolov8_encoder/resize/image', '/zed/zed_node/left/image_rect_color'),
    #                 ('detections_output', '/yolov8/detections')],

    # )

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
        input_tensor_name_arg,
        output_tensor_name_arg,


        zed_launch,
        yolov8_encoder_launch,
        ComposableNodeContainer(
            name=container_name,
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            # composable_node_descriptions=[yolov8_decoder_node, image_format_converter_node_left],
            composable_node_descriptions=[tensor_rt_node, yolov8_decoder_node, image_format_converter_node_left],
            # composable_node_descriptions=[image_format_converter_node_left],

            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO'],
            condition=UnlessCondition(run_standalone),
        ),
        # yolov8_visualizer_node,
    ])
