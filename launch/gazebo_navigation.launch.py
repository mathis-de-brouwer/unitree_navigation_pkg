#!/usr/bin/env python3
import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('unitree_navigation_pkg')
    turtlebot3_pkg = get_package_share_directory('turtlebot3_gazebo')
    
    # World file path
    world_path = os.path.join(pkg_dir, 'worlds', 'botopia_world.world')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(pkg_dir, 'models', 'botopiaRealCam.pt'),
        description='Path to YOLOv8 model file'
    )
    
    # Configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    model_path = LaunchConfiguration('model_path')
    
    # Set TurtleBot3 model environment variable (waffle_pi has a camera)
    os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'
    
    # Start Gazebo with our custom world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Spawn TurtleBot in our world
    spawn_turtlebot = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_pkg, 'launch', 'spawn_turtlebot3.launch.py')
        )
    )
    
    # Navigation node
    navigation_node = Node(
        package='unitree_navigation_pkg',
        executable='navigation_node',
        name='navigation_node',
        parameters=[{
            'constant_speed': 0.2,
            'turning_angle': 0.4,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # Camera bridge node - connects TurtleBot camera to YOLO segmentation
    camera_bridge = Node(
        package='unitree_navigation_pkg',
        executable='camera_bridge',
        name='camera_bridge',
        parameters=[{
            'model_path': model_path,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        model_path_arg,
        gazebo,
        spawn_turtlebot,
        navigation_node,
        camera_bridge,
    ])