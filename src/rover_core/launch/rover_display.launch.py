from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue 


def generate_launch_description():
    ld = LaunchDescription()

    description_package_share = FindPackageShare("rover_description")


# runs the display.launch file with the given arguments
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'rover_description',
            'rviz_config': PathJoinSubstitution([description_package_share,'config', 'rover.rviz']),
            'urdf_package_path': PathJoinSubstitution(['model', 'rover.urdf.xacro'])}.items()
    ))

    default_controller_config_path = PathJoinSubstitution([description_package_share, 'config', 'controllers.yaml'])
    ld.add_action(DeclareLaunchArgument(name='controller_config_path', default_value=default_controller_config_path,
                                        description='Absolute path to controller config file'))

    # ld.add_action(DeclareLaunchArgument(name='ros2_control_enable', default_value='false', 
    #                                     description='If true launch ros2_control_node'))

    robot_description_content = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([description_package_share, 'model', 'rover.urdf.xacro'])]),
        value_type=str
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {'robot_description': robot_description_content},
                LaunchConfiguration('controller_config_path')
            ],
            output="screen",
            # condition=IfCondition(LaunchConfiguration('ros2_control_enable'))
        )
    )

    return ld