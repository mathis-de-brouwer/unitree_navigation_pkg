#!/usr/bin/env python3
import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('unitree_navigation_pkg')
    
    # Default video path
    default_video_path = os.path.join(pkg_dir, 'models', 'videobotopia.mp4')
    
    # Set up the video path argument
    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value=default_video_path,
        description='Path to the video file for testing'
    )

    video_path = LaunchConfiguration('video_path')
    
    # Set up the model path argument
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(pkg_dir, 'models', 'botopiaRealCam.pt'),
        description='Path to the model file'
    )
    
    model_path = LaunchConfiguration('model_path')
    
    # Add a use_camera parameter 
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='false',  # Default to video file mode
        description='Whether to use live camera feed (true) or video file (false)'
    )
    
    use_camera = LaunchConfiguration('use_camera')

    return launch.LaunchDescription([
        video_path_arg,
        model_path_arg,
        use_camera_arg,
        
        # Navigation node - always active
        launch_ros.actions.Node(
            package='unitree_navigation_pkg',
            executable='navigation_node',
            name='navigation_node',
            parameters=[{
                'constant_speed': 0.2,
                'turning_angle': 0.4
            }]
        ),
        
        # Segmentation node - only run when using camera
        launch_ros.actions.Node(
            package='unitree_navigation_pkg',
            executable='segmentation_node',
            name='segmentation_node',
            parameters=[{'model_path': model_path}],
            condition=IfCondition(use_camera)
        ),
        
        # Video navigation tester - only run when not using camera
        launch_ros.actions.Node(
            package='unitree_navigation_pkg',
            executable='video_navigation_tester',
            name='video_navigation_tester',
            parameters=[{'video_path': video_path}],
            condition=UnlessCondition(use_camera)
        ),
        
        # Uncomment to use websocket server
        # launch_ros.actions.Node(
        #     package='unitree_navigation_pkg',
        #     executable='websocket_server',
        #     name='websocket_server'
        # ),
    ])
