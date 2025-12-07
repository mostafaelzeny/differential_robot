import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_config = LaunchConfiguration("slam_config")
    
    # Empty lifecycle nodes - no lifecycle management needed
    lifecycle_nodes = []
    
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )
    
    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("robot_slam"),
            "config",
            "slam_toolbox.yaml"
        ),
        description="Full path to slam toolbox YAML file"
    )
    
    # ----------------------------
    #    MAP SAVER SERVER NODE
    # ----------------------------
    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[
            {"save_map_timeout": 5.0},
            {"use_sim_time": use_sim_time},
            {"free_thresh_default": 0.196},
            {"occupied_thresh_default": 0.65}
        ]
    )
    
    # ----------------------------
    #        SLAM TOOLBOX NODE
    # ----------------------------
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",  # CHANGED: async for better performance
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {"use_sim_time": use_sim_time},
        ]
    )
    
    # Return LaunchDescription
    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        nav2_map_saver,  # RE-ENABLED: You need this to save maps
        slam_toolbox,
        # REMOVED: nav2_lifecycle_manager - not needed
    ])