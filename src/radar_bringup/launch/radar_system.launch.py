import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('radar_bringup')
    config_dir  = os.path.join(bringup_dir, 'config')

    camera_config   = os.path.join(config_dir, 'camera.yaml')
    pan_tilt_config = os.path.join(config_dir, 'pan_tilt.yaml')
    vitals_config   = os.path.join(config_dir, 'vitals.yaml')

    use_sim_vitals = LaunchConfiguration('use_sim_vitals')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_vitals',
            default_value='true',
            description='Use simulated vitals (true) or real MAX30102 sensor (false)',
        ),

        # --- Teleoperation ---
        Node(
            package='radar_teleop',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
        ),

        # --- Camera ---
        Node(
            package='radar_camera',
            executable='camera_node',
            name='camera_node',
            parameters=[camera_config],
            output='screen',
        ),

        # --- Pan-tilt (joystick-driven) ---
        Node(
            package='radar_pan_tilt',
            executable='pan_tilt_joy_node',
            name='pan_tilt_joy_node',
            parameters=[pan_tilt_config],
            output='screen',
        ),

        # --- Vitals: simulated (default) ---
        Node(
            package='radar_vitals',
            executable='vitals_node',
            name='vitals_node',
            parameters=[vitals_config],
            output='screen',
            condition=IfCondition(use_sim_vitals),
        ),

        # --- Vitals: real MAX30102 sensor ---
        Node(
            package='radar_vitals',
            executable='pulse_ox_node',
            name='pulse_ox_node',
            parameters=[vitals_config],
            output='screen',
            condition=UnlessCondition(use_sim_vitals),
        ),
    ])
