"""
Bring up the TB4 Ignition simulation, the semantic detector, Nav2 and RViz.

WORLD LOOKUP - IMPORTANT
    turtlebot4_ignition.launch.py resolves the world by NAME, searching
    IGN_GAZEBO_RESOURCE_PATH. It sets that variable with SetEnvironmentVariable
    (a full overwrite, not an append) and then starts Gazebo *inside the same
    include*, so there is no point in this file where we can inject our own
    worlds/ directory: setting it before the include gets overwritten, and setting
    it after is too late - Gazebo has already been spawned.

    Workaround: semantic_maze.sdf is symlinked into the TB4 worlds directory. See README.

ARGUMENT NAME COLLISION - WHY THE NAV2 SWITCH IS CALLED use_nav2
    turtlebot4_spawn.launch.py declares its own launch argument called 'nav2'.
    Launch arguments propagate down into includes, so an argument called 'nav2' here
    would leak into the TB4 include and make it start a SECOND, independent Nav2 stack
    with its own params - two of every node, fighting over the same topics and lifecycle
    transitions. The symptom is baffling: duplicate node names, "unknown goal response",
    and a global costmap that insists on a map frame we never configured.
    Ours is therefore called use_nav2, and TB4's is explicitly pinned to false below.

NO SLAM / NO LOCALIZATION
    Nothing publishes map->odom here, so nav2_params.yaml runs everything in the odom
    frame and drops static_layer. Launching turtlebot4_navigation's localization or
    slam alongside this would fight with those settings - see the header of
    nav2_params.yaml before changing this.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('semantic_nav_bringup')
    pkg_tb4_ignition = get_package_share_directory('turtlebot4_ignition_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    world = LaunchConfiguration('world')
    params_file = LaunchConfiguration('params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_rviz = LaunchConfiguration('use_rviz')

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

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_bringup, 'rviz', 'semantic_nav.rviz'),
        description='RViz config showing the costmaps, the plan and the laser scan.',
    )

    # Nav2 is switchable so the detector can be debugged on its own without paying for
    # the whole navigation stack - which matters here, the sim already runs at RTF ~0.44.
    declare_use_nav2 = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to bring up Nav2.',
    )

    # Same reasoning: RViz is not free on a 4GB GPU that is already rendering the sim.
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to bring up RViz.',
    )

    tb4_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb4_ignition, 'launch', 'turtlebot4_ignition.launch.py')
        ),
        launch_arguments={
            'world': world,
            # Pin TB4's own nav2 argument off - see the docstring. Without this we get
            # two Nav2 stacks.
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

    # RViz needs use_sim_time too, otherwise its TF lookups run against wall time while
    # everything else is on sim time, and displays flicker or vanish.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    # Undock to navigate properly
    undock = ExecuteProcess(
        cmd=['bash', '-c',
             'until ros2 topic echo /dock_status --once 2>/dev/null | grep -q "is_docked: true"; do sleep 2; done; '
             'ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"'],
        output='screen',
    )

    # After undocking the robot is still nose-to-nose with its dock, which then sits
    # squarely between the robot and any goal in front of it - the planner ends up
    # reacting to the dock rather than to the person, which would confound the A/B test.
    # Turning 180 degrees puts the dock behind the robot and leaves a clear corridor.
    turn_around = TimerAction(
        period=60.0,
        actions=[ExecuteProcess(
            cmd=['bash', '-c',
                 'ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '
                 '"{angular: {z: 0.5}}" & PID=$!; sleep 13; kill $PID; '
                 'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"'],
            output='screen',
        )],
    )

    return LaunchDescription([
        declare_world,
        declare_params,
        declare_nav2_params,
        declare_rviz_config,
        declare_use_nav2,
        declare_use_rviz,
        tb4_sim,
        detector,
        nav2,
        rviz,
        undock,
        turn_around
    ])