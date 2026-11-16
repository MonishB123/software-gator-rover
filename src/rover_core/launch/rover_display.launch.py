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

    ld.add_action(DeclareLaunchArgument(name='gui', default_value='true', description='enable gui of rviz2'))

# runs the display.launch file with the given arguments
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'rover_description',
            'rviz_config': PathJoinSubstitution([description_package_share,'config', 'rover.rviz']),
            'jsp_gui' : LaunchConfiguration('gui'),
            'urdf_package_path': PathJoinSubstitution(['model', 'rover.urdf.xacro'])}.items(),
    ))

    default_controller_config_path = PathJoinSubstitution([description_package_share, 'config', 'controllers.yaml'])
    ld.add_action(DeclareLaunchArgument(name='controller_config_path', default_value=default_controller_config_path,
                                        description='Absolute path to controller config file'))

    # ld.add_action(
    #     Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         name="controller_spawner",
    #         arguments=[
    #             "joint_state_broadcaster",
    #             "forward_position_controller",
    #             "--param-file",
    #             LaunchConfiguration('controller_config_path')
    #         ],
    #     ),
    # )

    # ld.add_action(IncludeLaunchDescription(
    #     PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py" ]),
    #     launch_arguments=[("gz_args", " -r -v 3 empty.sdf")],
    # ))

    # ld.add_action(
    #     Node(
    #         package="ros_gz_bridge",
    #         executable="parameter_bridge",
    #         arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    #         output="screen",
    #     )
    # )

    # ld.add_action(
    #     Node(
    #         package="ros_gz_sim",
    #         executable="create",
    #         output="screen",
    #         arguments=[
    #             "-topic",
    #             "/robot_description",
    #             "-allow_renaming",
    #             "true",
    #         ],
    #     )
    # )


    return ld