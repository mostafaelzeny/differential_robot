import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robot_description"),
            "launch",
            "ignition.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )
    
    # joystick = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("bumperbot_controller"),
    #         "launch",
    #         "joystick_teleop.launch.py"
    #     ),
    #     launch_arguments={
    #         "use_sim_time": "True"
    #     }.items()
    # )


    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
    )
    
    return LaunchDescription([
        gazebo,
        controller,
        # localization
        
        # joystick,
    ])