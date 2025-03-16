import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

DEFAULT_CONTAINER_NAME = 'zed_yolov8_container'

def generate_launch_description():
    config_file_common = os.path.join(
        get_package_share_directory('perception_setup'),
        'config',
        'zed_driver_params.yaml',
    )

    xacro_path = os.path.join(
        get_package_share_directory('zed_wrapper'), 'urdf', 'zed_descr.urdf.xacro'
    )

    image_format_converter_node_left = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_node_left',
        parameters=[{'encoding_desired': 'rgb8', 'image_width': 1280, 'image_height': 720}],
        remappings=[('image_raw', 'zed_node/left/image_rect_color'), ('image', 'image_rect')],
    )

    zed_wrapper_component = ComposableNode(
        package='zed_components',
        name='zed_node',
        plugin='stereolabs::ZedCamera',
        parameters=[config_file_common],
        remappings=[('zed_node/left/camera_info', '/camera_info_rect')],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='zed_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro', ' ', str(xacro_path), ' camera_name:=zed camera_model:=zed2i'])}],
    )

    tensor_rt_node = ComposableNode(
        name='tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[
            {'model_file_path': LaunchConfiguration('model_file_path'),
             'engine_file_path': LaunchConfiguration('engine_file_path'),
             'output_binding_names': LaunchConfiguration('output_binding_names'),
             'output_tensor_names': LaunchConfiguration('output_tensor_names'),
             'input_tensor_names': LaunchConfiguration('input_tensor_names'),
             'input_binding_names': LaunchConfiguration('input_binding_names'),
             'verbose': LaunchConfiguration('verbose'),
             'force_engine_update': LaunchConfiguration('force_engine_update')}],
    )

    yolov8_decoder_node = ComposableNode(
        name='yolov8_decoder_node',
        package='isaac_ros_yolov8',
        plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        parameters=[
            {'confidence_threshold': LaunchConfiguration('confidence_threshold'),
             'nms_threshold': LaunchConfiguration('nms_threshold'),
             'num_classes': 1}],
    )

    yolov8_encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('isaac_ros_dnn_image_encoder'), 'launch', 'dnn_image_encoder.launch.py')]),
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

    # Valve Detection Node (from file2)
    valve_detection_node = Node(
        package='valve_detection',
        executable='valve_detection_node',
        name='valve_detection_node',
        parameters=[os.path.join(get_package_share_directory('valve_detection'), 'config', 'valve_detection_params.yaml')],
        output='screen',
    )

    return LaunchDescription(
        [
            yolov8_encoder_launch,
            yolov8_visualizer_node,
            rsp_node,
            valve_detection_node,  # Adding valve detection node to the launch file
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
                ],
                output='screen',
                arguments=['--ros-args', '--log-level', 'INFO'],
            ),
        ]
    )
