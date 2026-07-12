"""
Bring up the TB4 Ignition simulation together with the semantic detector.

WORLD LOOKUP - IMPORTANT
    turtlebot4_ignition.launch.py resolves the world by NAME, searching
    IGN_GAZEBO_RESOURCE_PATH. It sets that variable with SetEnvironmentVariable
    (a full overwrite, not an append) and then starts Gazebo *inside the same
    include*, so there is no point in this file where we can inject our own
    worlds/ directory: setting it before the include gets overwritten, and
    setting it after is too late - Gazebo has already been spawned.

    Workaround: semantic_maze.sdf is symlinked into the TB4 worlds directory, so
    the stock resource path finds it. See README. The clean alternative is to stop
    including turtlebot4_ignition.launch.py and instead include ign_gazebo.launch.py,
    ros_ign_bridge.launch.py and the robot spawn separately, controlling the
    environment ourselves.

Nav2 is intentionally NOT launched yet - the semantic costmap layer does not exist.
Once it does, Nav2 gets added here so this file stays the single entry point.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('semantic_nav_bringup')
    pkg_tb4_ignition = get_package_share_directory('turtlebot4_ignition_bringup')

    world = LaunchConfiguration('world')
    params_file = LaunchConfiguration('params_file')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='semantic_maze',
        description='World name without .sdf. Must be resolvable on '
                    'IGN_GAZEBO_RESOURCE_PATH (see module docstring). '
                    'Use "maze" or "warehouse" for the stock worlds.',
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_bringup, 'params', 'semantic_detector.yaml'),
        description='Parameter file for the semantic detector.',
    )

    tb4_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb4_ignition, 'launch', 'turtlebot4_ignition.launch.py')
        ),
        launch_arguments={
            'world': world,
        }.items(),
    )

    # use_sim_time is in the params file, but is also forced here: if the params file
    # is ever swapped for one that omits it, the node would silently fall back to wall
    # time and every TF lookup would fail with a confusing extrapolation error.
    detector = Node(
        package='semantic_nav_detector',
        executable='semantic_detector_node',
        name='semantic_detector_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    return LaunchDescription([
        declare_world,
        declare_params,
        tb4_sim,
        detector,
    ])