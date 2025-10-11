import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # === Declare launch arguments ===
    model_file_path_arg = DeclareLaunchArgument(
        'model_file_path',
        default_value=os.path.join(
            get_package_share_directory('perception_setup'),
            'models',
            'unet-simple-320-240-l-5-e10-b16.onnx',
        ),
        description='Path to ONNX model file',
    )
    engine_file_path_arg = DeclareLaunchArgument(
        'engine_file_path',
        default_value=os.path.join(
            get_package_share_directory('perception_setup'),
            'models',
            'unet-simple-320-240-l-5-e10-b16.engine',
        ),
        description='Path to TensorRT engine file',
    )

    network_image_width_arg = DeclareLaunchArgument(
        'network_image_width', default_value='320'
    )
    network_image_height_arg = DeclareLaunchArgument(
        'network_image_height', default_value='240'
    )
    use_planar_input_arg = DeclareLaunchArgument(
        'use_planar_input', default_value='True'
    )
    encoder_image_mean_arg = DeclareLaunchArgument(
        'encoder_image_mean', default_value='[0.485, 0.456, 0.406]'
    )
    encoder_image_stddev_arg = DeclareLaunchArgument(
        'encoder_image_stddev', default_value='[0.229, 0.224, 0.225]'
    )
    alpha_arg = DeclareLaunchArgument('alpha', default_value='0.5')
    network_output_type_arg = DeclareLaunchArgument(
        'network_output_type',
        default_value='sigmoid',
        choices=['argmax', 'softmax', 'sigmoid'],
    )

    # === LaunchConfigurations ===
    model_file_path = LaunchConfiguration('model_file_path')
    engine_file_path = LaunchConfiguration('engine_file_path')
    network_image_width = LaunchConfiguration('network_image_width')
    network_image_height = LaunchConfiguration('network_image_height')
    use_planar_input = LaunchConfiguration('use_planar_input')
    encoder_image_mean = LaunchConfiguration('encoder_image_mean')
    encoder_image_stddev = LaunchConfiguration('encoder_image_stddev')
    alpha = LaunchConfiguration('alpha')
    network_output_type = LaunchConfiguration('network_output_type')

    # === Image Processing and Encoder Nodes ===

    debayer_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::DebayerNode',
        name='debayer_node',
        remappings=[
            ('image_raw', '/gripper_camera/image_raw'),
            ('image_color', '/debayered_image'),
        ],
    )

    image_format_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_node',
        parameters=[{'encoding_desired': 'rgb8'}],
        remappings=[
            ('image_raw', '/debayered_image'),
            ('image', '/image_rect'),
        ],
    )

    resize_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        name='resize_node',
        parameters=[
            {
                'output_width': LaunchConfiguration('network_image_width'),
                'output_height': LaunchConfiguration('network_image_height'),
                'keep_aspect_ratio': True,
                'disable_padding': False,
                'encoding_desired': 'rgb8',
            }
        ],
        remappings=[
            ('/image', '/image_rect'),
            ('/resize/image', '/image_resized'),
            (
                '/camera_info',
                '/gripper_camera/camera_info',
            ),  # resize node won't publish without camera info
        ],
    )

    image_to_tensor = ComposableNode(
        package='isaac_ros_tensor_proc',
        plugin='nvidia::isaac_ros::dnn_inference::ImageToTensorNode',
        name='image_to_tensor',
        parameters=[{'tensor_name': 'image', 'scale': True}],
        remappings=[
            ('image', '/image_resized'),
            ('tensor', '/unet_encoder/image_tensor'),
        ],
    )

    normalize_node = ComposableNode(
        package='isaac_ros_tensor_proc',
        plugin='nvidia::isaac_ros::dnn_inference::ImageTensorNormalizeNode',
        name='normalize_node',
        parameters=[
            {
                'mean': LaunchConfiguration('encoder_image_mean'),
                'stddev': LaunchConfiguration('encoder_image_stddev'),
                'input_tensor_name': 'image',
                'output_tensor_name': 'image',
            }
        ],
        remappings=[
            ('tensor', '/unet_encoder/image_tensor'),
            ('normalized_tensor', '/unet_encoder/normalized_tensor'),
        ],
    )

    interleaved_to_planar_node = ComposableNode(
        condition=IfCondition(LaunchConfiguration('use_planar_input')),
        package='isaac_ros_tensor_proc',
        plugin='nvidia::isaac_ros::dnn_inference::InterleavedToPlanarNode',
        name='interleaved_to_planar_node',
        parameters=[
            {
                'input_tensor_shape': [
                    LaunchConfiguration('network_image_height'),
                    LaunchConfiguration('network_image_width'),
                    3,
                ],
                'num_blocks': 40,
            }
        ],
        remappings=[
            ('interleaved_tensor', '/unet_encoder/normalized_tensor'),
            ('planar_tensor', '/unet_encoder/planar_tensor'),
        ],
    )

    reshape_node = ComposableNode(
        package='isaac_ros_tensor_proc',
        plugin='nvidia::isaac_ros::dnn_inference::ReshapeNode',
        name='reshape_node',
        parameters=[
            {
                'input_tensor_shape': [
                    3,
                    LaunchConfiguration('network_image_height'),
                    LaunchConfiguration('network_image_width'),
                ],
                'output_tensor_shape': [
                    1,
                    3,
                    LaunchConfiguration('network_image_height'),
                    LaunchConfiguration('network_image_width'),
                ],
                'output_tensor_name': 'input_tensor',
                'num_blocks': 40,
            }
        ],
        remappings=[
            ('tensor', '/unet_encoder/planar_tensor'),
            ('reshaped_tensor', '/unet_encoder/reshaped_tensor'),
        ],
    )

    # === TensorRT and Decoder ===

    tensor_rt_node = ComposableNode(
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        name='tensor_rt',
        parameters=[
            {
                'model_file_path': LaunchConfiguration('model_file_path'),
                'engine_file_path': LaunchConfiguration('engine_file_path'),
                'input_tensor_names': ['input_tensor'],
                'input_binding_names': ['input'],
                'input_tensor_formats': ['nitros_tensor_list_nchw_rgb_f32'],
                'output_tensor_names': ['output_tensor'],
                'output_binding_names': ['output'],
                'output_tensor_formats': ['nitros_tensor_list_nchw_rgb_f32'],
                'verbose': True,
                'force_engine_update': False,
            }
        ],
        remappings=[
            ('tensor_pub', '/unet_encoder/reshaped_tensor'),
            ('tensor_sub', '/unet_prediction'),
        ],
    )

    unet_decoder_node = ComposableNode(
        name='unet_decoder',
        package='isaac_ros_unet',
        plugin='nvidia::isaac_ros::unet::UNetDecoderNode',
        parameters=[
            {
                'network_output_type': network_output_type,
                'color_segmentation_mask_encoding': 'rgb8',
                'mask_width': network_image_width,
                'mask_height': network_image_height,
                'color_palette': [
                    0x556B2F,
                    0x800000,
                    0x008080,
                    0x000080,
                    0x9ACD32,
                    0xFF0000,
                    0xFF8C00,
                    0xFFD700,
                    0x00FF00,
                    0xBA55D3,
                    0x00FA9A,
                    0x00FFFF,
                    0x0000FF,
                    0xF08080,
                    0xFF00FF,
                    0x1E90FF,
                    0xDDA0DD,
                    0xFF1493,
                    0x87CEFA,
                    0xFFDEAD,
                ],
            }
        ],
        remappings=[
            ('/tensor_sub', '/unet_prediction'),
            ('/unet/colored_segmentation_mask', '/unet/colored_segmentation_mask'),
            ('/unet/raw_segmentation_mask', '/unet/raw_segmentation_mask'),
        ],
    )

    alpha_blend_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::AlphaBlendNode',
        name='alpha_blend',
        parameters=[
            {
                'alpha': alpha,
                'mask_queue_size': 5,
                'image_queue_size': 5,
                'sync_queue_size': 5,
            }
        ],
        remappings=[
            ('mask_input', '/unet/colored_segmentation_mask'),
            ('image_input', '/image_resized'),
            ('blended_image', '/segmentation_image_overlay'),
        ],
    )

    # === Container ===

    container = ComposableNodeContainer(
        name='unet_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            debayer_node,
            image_format_node,
            resize_node,
            image_to_tensor,
            normalize_node,
            interleaved_to_planar_node,
            reshape_node,
            tensor_rt_node,
            unet_decoder_node,
            alpha_blend_node,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    return LaunchDescription(
        [
            model_file_path_arg,
            engine_file_path_arg,
            network_image_width_arg,
            network_image_height_arg,
            encoder_image_mean_arg,
            encoder_image_stddev_arg,
            use_planar_input_arg,
            alpha_arg,
            network_output_type_arg,
            container,
        ]
    )
