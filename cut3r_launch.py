#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'cut3r_path',
            default_value='/home/race10/CUT3R',
            description='Path to CUT3R installation'
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/race10/CUT3R/src/cut3r_512_dpt_4_64.pth',
            description='Path to CUT3R model file'
        ),
        DeclareLaunchArgument(
            'publish_current_pointcloud',
            default_value='true',
            description='Whether to publish current frame point cloud'
        ),
        DeclareLaunchArgument(
            'publish_aggregated_pointcloud',
            default_value='true',
            description='Whether to publish aggregated point cloud'
        ),
        DeclareLaunchArgument(
            'publish_depth_map',
            default_value='true',
            description='Whether to publish depth map'
        ),
        DeclareLaunchArgument(
            'voxel_size',
            default_value='0.05',
            description='Voxel size for point cloud downsampling'
        ),
        DeclareLaunchArgument(
            'enable_colors',
            default_value='true',
            description='Enable color extraction from images'
        ),
        DeclareLaunchArgument(
            'max_accumulated_frames',
            default_value='72',
            description='Maximum frames to accumulate before clearing'
        ),
        DeclareLaunchArgument(
            'circle_radius',
            default_value='1.0',
            description='Radius of circular motion for 360° capture'
        ),
        DeclareLaunchArgument(
            'rotation_speed',
            default_value='5.0',
            description='Degrees per frame for rotation'
        ),
        
        # CUT3R Processor Node
        Node(
            package='cut3r_ros',  # Replace with your actual package name
            executable='cut3r_cycle_working.py',
            name='cut3r_processor',
            output='screen',
            parameters=[{
                'cut3r_path': LaunchConfiguration('cut3r_path'),
                'model_path': LaunchConfiguration('model_path'),
                'publish_current_pointcloud': LaunchConfiguration('publish_current_pointcloud'),
                'publish_aggregated_pointcloud': LaunchConfiguration('publish_aggregated_pointcloud'),
                'publish_depth_map': LaunchConfiguration('publish_depth_map'),
                'voxel_size': LaunchConfiguration('voxel_size'),
                'enable_colors': LaunchConfiguration('enable_colors'),
                'max_accumulated_frames': LaunchConfiguration('max_accumulated_frames'),
                'circle_radius': LaunchConfiguration('circle_radius'),
                'rotation_speed': LaunchConfiguration('rotation_speed'),
            }],
            remappings=[
                ('/race10/cam1/color/image_raw', '/camera/color/image_raw'),  # Adjust topic name as needed
            ]
        )
    ])