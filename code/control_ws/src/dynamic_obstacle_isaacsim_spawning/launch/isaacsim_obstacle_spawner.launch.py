#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ali Jafari Fesharaki

"""
Launch file for Isaac Sim dynamic obstacle spawning with ros2_control support.

This launch file:
1. Starts the Isaac Sim spawner node
2. Spawns ros2_control controllers (joint_state_broadcaster, position_controller)
3. Configures trajectory publishers for dynamic obstacles
4. Sets up communication with Isaac Sim via ROS2 bridge

Usage:
    ros2 launch dynamic_obstacle_isaacsim_spawning isaacsim_obstacle_spawner.launch.py
    
    # With custom configuration:
    ros2 launch dynamic_obstacle_isaacsim_spawning isaacsim_obstacle_spawner.launch.py \\
        config_file:=/path/to/obstacles.yaml
    
    # Using ros2_control mode (similar to Gazebo):
    ros2 launch dynamic_obstacle_isaacsim_spawning isaacsim_obstacle_spawner.launch.py \\
        use_ros2_control:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
import yaml


def generate_ros2_control_nodes(context, *args, **kwargs):
    """Generate ros2_control nodes for each dynamic obstacle."""
    
    package_dir = get_package_share_directory('dynamic_obstacle_isaacsim_spawning')
    config_file = LaunchConfiguration('config_file').perform(context)
    trajectory_output_dir = LaunchConfiguration('trajectory_output_dir').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    use_ros2_control = LaunchConfiguration('use_ros2_control').perform(context)
    
    nodes = []
    
    if use_ros2_control != 'true':
        return nodes
    
    if not os.path.exists(config_file):
        return nodes
    
    # Load configuration
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    obstacles = config.get('obstacles', [])
    obstacle_controllers = os.path.join(package_dir, 'config', 'obstacles_controller.yaml')
    
    for obstacle in obstacles:
        if not obstacle.get('enabled', True):
            continue
        
        if not obstacle.get('motion'):
            continue
        
        name = obstacle['name']
        trajectory_file = os.path.join(trajectory_output_dir, f"{name}_trajectory.yaml")
        
        # Joint state broadcaster spawner
        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            namespace=name,
            parameters=[{'use_sim_time': use_sim_time == 'true'}],
            arguments=[
                'joint_state_broadcaster',
                '-c', f'/{name}/controller_manager'
            ],
            output='screen'
        )
        
        # Position controller spawner
        position_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            namespace=name,
            parameters=[{'use_sim_time': use_sim_time == 'true'}],
            arguments=[
                'position_controller',
                '--param-file', obstacle_controllers,
                '-c', f'/{name}/controller_manager'
            ],
            output='screen'
        )
        
        # Trajectory publisher node (uses JointTrajectory mode)
        trajectory_node = Node(
            package='dynamic_obstacle_isaacsim_spawning',
            executable='trajectory_publisher',
            name=f'{name}_trajectory_publisher',
            namespace=name,
            parameters=[{
                'trajectory_file': trajectory_file,
                'controller_topic': 'position_controller/joint_trajectory',
                'use_pose_mode': False,  # Use JointTrajectory mode for ros2_control
                'use_sim_time': use_sim_time == 'true'
            }],
            output='screen'
        )
        
        nodes.extend([joint_state_broadcaster_spawner, position_controller_spawner, trajectory_node])
    
    return nodes


def generate_pose_mode_nodes(context, *args, **kwargs):
    """Generate trajectory publisher nodes in pose mode (non-ros2_control)."""
    
    package_dir = get_package_share_directory('dynamic_obstacle_isaacsim_spawning')
    config_file = LaunchConfiguration('config_file').perform(context)
    trajectory_output_dir = LaunchConfiguration('trajectory_output_dir').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    use_ros2_control = LaunchConfiguration('use_ros2_control').perform(context)
    
    nodes = []
    
    if use_ros2_control == 'true':
        return nodes
    
    if not os.path.exists(config_file):
        return nodes
    
    # Load configuration
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    obstacles = config.get('obstacles', [])
    
    for obstacle in obstacles:
        if not obstacle.get('enabled', True):
            continue
        
        if not obstacle.get('motion'):
            continue
        
        name = obstacle['name']
        trajectory_file = os.path.join(trajectory_output_dir, f"{name}_trajectory.yaml")
        
        trajectory_node = Node(
            package='dynamic_obstacle_isaacsim_spawning',
            executable='trajectory_publisher',
            name=f'{name}_trajectory_publisher',
            namespace=name,
            parameters=[{
                'trajectory_file': trajectory_file,
                'controller_topic': 'target_pose',
                'use_pose_mode': True,  # Use PoseStamped mode
                'use_sim_time': use_sim_time == 'true'
            }],
            output='screen'
        )
        
        nodes.append(trajectory_node)
    
    return nodes


def generate_launch_description():
    """Generate launch description for Isaac Sim obstacle spawning."""
    
    package_dir = get_package_share_directory('dynamic_obstacle_isaacsim_spawning')
    
    # Default configuration paths
    default_config = os.path.join(package_dir, 'config', 'obstacles.yaml')
    default_controllers_config = os.path.join(package_dir, 'config', 'obstacles_controller.yaml')
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to obstacles YAML configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='30.0',
        description='Motion controller update rate in Hz'
    )
    
    trajectory_output_dir_arg = DeclareLaunchArgument(
        'trajectory_output_dir',
        default_value='/tmp',
        description='Directory to output trajectory YAML files'
    )
    
    spawn_script_output_arg = DeclareLaunchArgument(
        'spawn_script_output',
        default_value='/tmp/isaacsim_spawn.py',
        description='Output path for generated spawn script'
    )
    
    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control with joint_trajectory_controller (true) or pose-based control (false)'
    )
    
    controllers_config_arg = DeclareLaunchArgument(
        'controllers_config',
        default_value=default_controllers_config,
        description='Path to ros2_controllers.yaml configuration file'
    )
    
    # Isaac Sim Spawner Node
    spawner_node = Node(
        package='dynamic_obstacle_isaacsim_spawning',
        executable='isaacsim_spawner',
        name='isaacsim_spawner',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'controllers_config': LaunchConfiguration('controllers_config'),
            'spawn_script_output': LaunchConfiguration('spawn_script_output'),
            'trajectory_output_dir': LaunchConfiguration('trajectory_output_dir'),
            'update_rate': LaunchConfiguration('update_rate'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    )
    
    # Log configuration info
    log_info = LogInfo(
        msg=[
            'Isaac Sim Obstacle Spawner launched with config: ',
            LaunchConfiguration('config_file'),
            ' | ros2_control: ',
            LaunchConfiguration('use_ros2_control')
        ]
    )
    
    # Delayed trajectory node generation (wait for spawner to initialize)
    # ros2_control mode nodes
    ros2_control_nodes = TimerAction(
        period=5.0,
        actions=[OpaqueFunction(function=generate_ros2_control_nodes)]
    )
    
    # Pose mode nodes (non-ros2_control)
    pose_mode_nodes = TimerAction(
        period=2.0,
        actions=[OpaqueFunction(function=generate_pose_mode_nodes)]
    )
    
    return LaunchDescription([
        # Launch arguments
        config_file_arg,
        use_sim_time_arg,
        update_rate_arg,
        trajectory_output_dir_arg,
        spawn_script_output_arg,
        use_ros2_control_arg,
        controllers_config_arg,
        
        # Log info
        log_info,
        
        # Spawner node
        spawner_node,
        
        # Trajectory publishers (ros2_control or pose mode)
        ros2_control_nodes,
        pose_mode_nodes,
    ])

