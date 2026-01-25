from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='radar_teleop', executable='teleop_node'),
        Node(package='radar_camera', executable='camera_node'),
        Node(package='radar_pan_tilt', executable='pan_tilt_node'),
        Node(package='radar_vitals', executable='vitals_node'),
    ])
