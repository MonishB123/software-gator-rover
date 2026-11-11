"""
apriltag.launch.py - entry point for other teams.

Standalone:
  ros2 launch rover_vision apriltag.launch.py
  ros2 launch rover_vision apriltag.launch.py tag_size_m:=0.16 image_topic:=/camera/image_raw

With a fake camera (no hardware):
  ros2 launch rover_vision apriltag.launch.py video_path:=/workspace/src/rover_vision/test_images/480p_test_video.mp4.mov

From another package's launch file:
  IncludeLaunchDescription(PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('rover_vision'), 'launch', 'apriltag.launch.py')),
      launch_arguments={'image_topic': '/camera/image_raw'}.items())
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    tag_size_m = LaunchConfiguration('tag_size_m')
    hfov_deg = LaunchConfiguration('hfov_deg')
    publish_debug_image = LaunchConfiguration('publish_debug_image')
    video_path = LaunchConfiguration('video_path')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('tag_size_m', default_value='0.16'),
        DeclareLaunchArgument('hfov_deg', default_value='70.0',
                              description='Fallback FOV used until CameraInfo arrives'),
        DeclareLaunchArgument('publish_debug_image', default_value='false'),
        DeclareLaunchArgument('video_path', default_value='',
                              description='If set, publish this video as a fake camera'),

        Node(
            package='rover_vision',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'camera_info_topic': camera_info_topic,
                'tag_size_m': tag_size_m,
                'hfov_deg': hfov_deg,
                'publish_debug_image': publish_debug_image,
            }],
        ),

        Node(
            package='rover_vision',
            executable='video_publisher',
            name='video_publisher',
            output='screen',
            parameters=[{'video_path': video_path, 'topic': image_topic}],
            condition=IfCondition(PythonExpression(["'", video_path, "' != ''"])),
        ),
    ])