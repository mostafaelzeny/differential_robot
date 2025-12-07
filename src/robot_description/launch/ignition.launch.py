import os
from os import pathsep
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression

def generate_launch_description():
    robot_description_pkg = get_package_share_directory("robot_description")
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=str(Path(robot_description_pkg) / "urdf" / "robo_description.urdf.xacro"),
        description="Absolute path to robot URDF/XACRO file"
    )
    
    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="small_house")
    
    world_path = PathJoinSubstitution([
        robot_description_pkg,
        "worlds",
        PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
    ])
    
    model_path = str(Path(robot_description_pkg).parent.resolve())
    model_path += pathsep + os.path.join(robot_description_pkg, 'models')
    
    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
    )
    
    robot_description_content = {
        "robot_description": ParameterValue(
            Command(["xacro ", LaunchConfiguration("model")]),
            value_type=str
        )
    }
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description_content, {"use_sim_time": True}]
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
        }.items()
    )
    
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "my_robot"
        ]
    )
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(robot_description_pkg, 'config', 'diff_drive.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    
    return LaunchDescription([
        model_arg,
        world_name_arg,
        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge
    ])