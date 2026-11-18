from launch import LaunchDescription
import os

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue 
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit



def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        "gui", default_value="false", choices=["true", "false"],
        description="Start the Gazebo GUI and RViz; false runs headless",
    ))
    gui = LaunchConfiguration("gui")
    ld.add_action(SetLaunchConfiguration("gazebo_headless_args", ""))
    ld.add_action(SetLaunchConfiguration(
        "gazebo_headless_args", "-s --headless-rendering ",
        condition=UnlessCondition(gui),
    ))

    description_package_share = FindPackageShare("rover_description")

    # Support Fortress and newer Gazebo while preserving custom resource paths.
    for variable in ("IGN_GAZEBO_RESOURCE_PATH", "GZ_SIM_RESOURCE_PATH"):
        ld.add_action(SetEnvironmentVariable(variable, [
            PathJoinSubstitution([description_package_share, "models"]),
            os.pathsep,
            EnvironmentVariable(variable, default_value=""),
        ]))

    default_controller_config_path = PathJoinSubstitution([description_package_share, 'config', 'controllers.yaml'])
    ld.add_action(DeclareLaunchArgument(name='controller_config_path', default_value=default_controller_config_path,
                                        description='Absolute path to controller config file'))

# runs the display.launch file with the given arguments
    # ld.add_action(IncludeLaunchDescription(
    #     PathJoinSubstitution([FindPackageShare('rover_core'), 'launch', 'rover_display.launch.py']),
    #     launch_arguments={
    #         'gui' : 'false'
    #     }
    # ))

    robot_description = ParameterValue(Command([
        "xacro",
        " ",
        PathJoinSubstitution([
            description_package_share,
            "model",
            "rover.urdf.xacro",
        ]),
    ]), value_type=str)
    
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    'use_sim_time': True
                }
            ],
        ),
    )

    default_sdf_world_path = PathJoinSubstitution([description_package_share, 'worlds', 'lunabotics_arena.world'])
    ld.add_action(DeclareLaunchArgument(name='world_sdf_path', default_value=default_sdf_world_path, description='absolute path to the sdf world file'))

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ])
            ),
            launch_arguments={
                "gz_args": [
                    "-r -v 3 ", LaunchConfiguration("gazebo_headless_args"),
                    LaunchConfiguration("world_sdf_path"),
                ],
            }.items(),
        )
    )

    ld.add_action(
        Node(
            package="ros_gz_bridge",
            parameters=[{'use_sim_time' : True}],
            executable="parameter_bridge",
            # The [ separator bridges Gazebo messages into ROS only.
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                "/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
            output="screen",
        )
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        parameters=[{"use_sim_time": True}],
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "rover",
            "-allow_renaming", "true",
            "-x", "2.0",
            "-z", "0.3",
        ],
    )
    
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": True}],
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
    )
    
    drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": True}],
        output="screen",
        arguments=[
            "diff_drive_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
    )

    # Register handlers before starting the process they watch.
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_spawner],
        )
    ))
    
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[drive_spawner],
        )
    ))
 
    ld.add_action(spawn_robot)



    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            condition=IfCondition(gui),
            parameters=[{"use_sim_time": True}],
            output="log",
            arguments=[
                "-d",
                PathJoinSubstitution([description_package_share, 'config', 'rover.rviz'])
            ],
        )
    )


    return ld