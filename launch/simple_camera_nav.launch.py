import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('unitree_navigation_pkg')
    
    # Launch arguments
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera ID to use (typically 0 for built-in webcam)'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(pkg_dir, 'models', 'botopiaRealCam.pt'),
        description='Path to YOLOv8 model file'
    )
    
    cmd_topic_arg = DeclareLaunchArgument(
        'cmd_topic',
        default_value='/cmd_vel',
        description='Topic for velocity commands'
    )
    
    # Set TurtleBot model environment variable
    os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'
    
    # Camera bridge node with remapping
    camera_bridge = Node(
        package='unitree_navigation_pkg',
        executable='camera_bridge',
        name='camera_bridge',
        parameters=[{
            'camera_id': PythonExpression(["int('", LaunchConfiguration('camera_id'), "')"]),
            'model_path': LaunchConfiguration('model_path'),
        }],
        remappings=[
            # Use LaunchConfiguration to allow overriding the topic
            ('/cmd_vel', LaunchConfiguration('cmd_topic')),
        ],
        output='screen'
    )
    
    # Navigation node with remapping
    navigation_node = Node(
        package='unitree_navigation_pkg',
        executable='navigation_node',
        name='navigation_node',
        parameters=[{
            'constant_speed': 0.2,
            'turning_angle': 0.4
        }],
        remappings=[
            # Use the same topic as camera_bridge
            ('/cmd_vel', LaunchConfiguration('cmd_topic')),
        ],
        output='screen'
    )

    return LaunchDescription([
        camera_id_arg,
        model_path_arg,
        cmd_topic_arg,
        camera_bridge,
        navigation_node,
    ])