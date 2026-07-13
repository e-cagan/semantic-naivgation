"""
Bring up the TB4 Ignition simulation, the semantic detector, and Nav2.

WORLD LOOKUP - IMPORTANT
    turtlebot4_ignition.launch.py resolves the world by NAME, searching
    IGN_GAZEBO_RESOURCE_PATH. It sets that variable with SetEnvironmentVariable
    (a full overwrite, not an append) and then starts Gazebo *inside the same
    include*, so there is no point in this file where we can inject our own
    worlds/ directory: setting it before the include gets overwritten, and setting
    it after is too late - Gazebo has already been spawned.

    Workaround: semantic_maze.sdf is symlinked into the TB4 worlds directory. See README.

NO SLAM / NO LOCALIZATION
    Nothing publishes map->odom here, so nav2_params.yaml runs everything in the odom
    frame and drops static_layer. Launching turtlebot4_navigation's localization or
    slam alongside this would fight with those settings - see the header of
    nav2_params.yaml before changing this.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('semantic_nav_bringup')
    pkg_tb4_ignition = get_package_share_directory('turtlebot4_ignition_bringup')
    pkg_tb4_navigation = get_package_share_directory('turtlebot4_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    world = LaunchConfiguration('world')
    params_file = LaunchConfiguration('params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    use_nav2 = LaunchConfiguration('use_nav2')

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

    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(pkg_bringup, 'params', 'nav2_params.yaml'),
        description='Nav2 parameter file (odom-frame, semantic layer enabled).',
    )

    # Nav2 is switchable so the detector can be debugged on its own without paying for
    # the whole navigation stack - which matters here, the sim already runs at RTF ~0.44.
    declare_nav2 = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to bring up Nav2.',
    )

    tb4_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb4_ignition, 'launch', 'turtlebot4_ignition.launch.py')
        ),
        launch_arguments={
            'world': world,
            # TB4's spawn launch has its OWN 'nav2' argument. Without pinning it to
            # false, our nav2 argument leaks into the include and TB4 starts a second,
            # independent Nav2 stack with its own params - two of every node, fighting
            # over the same topics and lifecycle transitions.
            'nav2': 'false',
        }.items(),
    )

    # use_sim_time is in the params file, but is forced here too: if the params file is
    # ever swapped for one that omits it, the node would silently fall back to wall time
    # and every TF lookup would fail with a confusing extrapolation error.
    detector = Node(
        package='semantic_nav_detector',
        executable='semantic_detector_node',
        name='semantic_detector_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    # The semantic costmap layer is not launched as a node - pluginlib loads it into the
    # Nav2 costmap processes from nav2_params.yaml. Nothing to start here; if the layer
    # is missing, Nav2 will complain that the plugin "does not exist" at startup.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': 'true',
            'use_composition': 'False',
        }.items(),
        condition=IfCondition(use_nav2),
    )

    return LaunchDescription([
        declare_world,
        declare_params,
        declare_nav2_params,
        declare_nav2,
        tb4_sim,
        detector,
        nav2,
    ])