import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Set the TURTLEBOT3_MODEL environment variable
    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    # Get the share directory for our own package
    pkg_10xassign = get_package_share_directory('10xassign')
    
    # Get the share directory for turtlebot3_gazebo
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # --- 1. Launch the Turtlebot3 Gazebo simulation ---
    turtlebot3_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # --- 2. Launch RViz ---
    rviz_config_file = os.path.join(pkg_10xassign, 'rviz', 'dwa_config.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # --- 3. Launch Our DWA Planner Node ---
    dwa_planner_node = Node(
        package='10xassign',
        executable='dwa_planner_node',
        name='dwa_planner_node',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'w_goal_dist': 3.0},
                    {'w_obstacle': 0.1},
                    {'w_velocity': 0.05}
        ]
    )

    return LaunchDescription([
        turtlebot3_sim_launch,
        rviz_node,
        dwa_planner_node
    ])