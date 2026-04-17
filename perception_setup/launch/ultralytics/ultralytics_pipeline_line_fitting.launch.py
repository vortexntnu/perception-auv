
import os
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
    yolo_params = os.path.join(
        get_package_share_directory('vortex_image_segmentation'),
        'params',
        'yolo_params.yaml',
    )

    irls_config = os.path.join(
        get_package_share_directory('irls_line_fitter_2x'),
        'config', 
        'irls_line.yaml'
    )
 
    return LaunchDescription([
        # YOLO segmentation node
        Node(
            package='vortex_image_segmentation',
            executable='yolo_seg_node',
            name='yolo_segmentation_node',
            output='screen',
            parameters=[yolo_params],
        ),
 
        # YOLO classifier node
        Node(
            package='vortex_yolo_classifiy',
            executable='classifier_node',
            name='classifier_node',
            output='screen',
            parameters=[
                {
                    'model_path': '/home/gard/ros2_ws/src/vortex-deep-learning-pipelines/runs/classify/results/classify-20260304-154252/weights/best.pt'
                }
            ],
        ),
 
        # IRLS line fitter node
        Node(
            package='irls_line_fitter_2x',
            executable='irls_line_node',
            name='irls_line_node',
            parameters=[irls_config],
        ),
 
        # Image to GStreamer node
        Node(
            package='image_to_gstreamer',
            executable='image_to_gstreamer_node',
            name='image_to_gstreamer_node',
            output='screen',
            parameters=[
            {
                'input_topic': "/irls_line/image",
                'host': '10.0.0.169',
                'port': 5001,
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
        ),
    ])