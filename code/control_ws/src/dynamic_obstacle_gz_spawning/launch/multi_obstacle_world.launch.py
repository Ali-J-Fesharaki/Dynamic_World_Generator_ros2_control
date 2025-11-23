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
# Authors: Arshad Mehmood

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable ,IncludeLaunchDescription,LogInfo
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch.substitutions import EnvironmentVariable 

from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import OpaqueFunction
from multi_object_scripts.utils import ObstacleLoader
import psutil


def gz_bringup(context, *args, **kwargs):
    # Check if gz sim is already running
    gz_running = False
    for proc in psutil.process_iter(['name', 'cmdline']):
        try:
            if proc.info['name'] and 'ruby' in proc.info['name']:
                cmdline = proc.info.get('cmdline', [])
                if any('gz' in arg or 'ign' in arg for arg in cmdline):
                    gz_running = True
                    break
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass

    if gz_running:
        print("Gazebo/Ignition appears to be running already. Skipping launch of gz_sim.")
        return []

    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    world_name = LaunchConfiguration('world_name', default='final_scenario').perform(context)
    dynamic_obstacle_gz_spawning_pkg = get_package_share_directory('dynamic_obstacle_gz_spawning')
    world_path = os.path.join(dynamic_obstacle_gz_spawning_pkg, 'worlds', f'{world_name}.sdf')
        # Launch Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s -v2 {world_path}', 'on_exit_shutdown': 'true'}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v2', 'on_exit_shutdown': 'true'}.items()
    )
    return [gzserver_cmd, gzclient_cmd]
def generate_launch_description():
    dynamic_obstacle_gz_spawning_dir = get_package_share_directory('dynamic_obstacle_gz_spawning')
    
    # Configuration paths
    obstacle_config_path = os.path.join(dynamic_obstacle_gz_spawning_dir, 'config', 'obstacles.yaml')
    obstacles_controller_path = os.path.join(dynamic_obstacle_gz_spawning_dir, 'config', 'obstacles_controller.yaml')
    
    # Initialize ObstacleLoader
    obstacle_loader = ObstacleLoader(
        config_path=obstacle_config_path,
        sim_version='fortress',
        controllers_config_path=obstacles_controller_path
    )
    
    # Load obstacle configurations
    obstacle_loader.load_config()
    obstacles = obstacle_loader.get_obstacle_list()
    combined_paths = f"{os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')}"
    # Simulation config
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Main LaunchDescription
    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=combined_paths))
    ld.add_action(
    LogInfo(msg=['IGN_GAZEBO_RESOURCE_PATH = ', EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH')]))
    ld.add_action(OpaqueFunction(function=gz_bringup))

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    frame_prefix = LaunchConfiguration('frame_prefix', default='')
    obstacle_controllers = PathJoinSubstitution(
        [
            dynamic_obstacle_gz_spawning_dir,
            'config',
            'obstacles_controller.yaml',
        ]
    )

    # Generate trajectories
    trajectory_output_dir = os.path.join('/tmp')
    obstacle_loader.generate_trajectory_yaml(trajectory_output_dir)

    for obstacle in obstacles:
        namespace = obstacle['name']
        has_motion = obstacle.get('has_motion', False)
        
        # Generate xacro content for this specific obstacle
        obstacle_xacro_content = obstacle_loader.generate_obstacle_xacro(obstacle)
        
        # Save to temporary file for processing
        temp_xacro_path = os.path.join('/tmp', f'{namespace}_obstacle.xacro')
        with open(temp_xacro_path, 'w') as f:
            f.write(obstacle_xacro_content)

        obstacle_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),' ',temp_xacro_path,' ',f'namespace:={namespace}'
        ]
        )

        if has_motion:
            obstacle_state_publisher = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=namespace,
                remappings=remappings,
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': obstacle_description_content,
                    'frame_prefix': PythonExpression(["'", frame_prefix, "/'"])
                }])
            spawner_node = Node(
                package='ros_gz_sim',
                executable='create',
                namespace=namespace,
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    '-name', f'{namespace}',
                    '-topic', 'robot_description',
                    '-allow_renaming', 'true',
                    '-x', str(obstacle['x_pose']),
                    '-y', str(obstacle['y_pose']),
                    '-z', '0.5',
                ],
                output='screen',
            )
            joint_state_broadcaster_spawner =Node(
                package='controller_manager',
                executable='spawner',
                namespace=namespace,
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['joint_state_broadcaster',            
                        '-c',f'/{namespace}/controller_manager'],

            )
            position_controller_controller_spawner = Node(
                package='controller_manager',
                executable='spawner',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    'position_controller',
                    '--param-file',
                    obstacle_controllers,
                    '-c', f'/{namespace}/controller_manager'
                    ],

            )
            joint_state_cmd=RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawner_node,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            )
            drive_controller_cmd=RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[position_controller_controller_spawner],
                )
            )

            trajectory_publisher_node = Node(
                package='dynamic_obstacle_gz_spawning',
                executable='trajectory_publisher',
                name='trajectory_publisher',
                namespace=namespace,
                parameters=[{
                    'trajectory_file': os.path.join(trajectory_output_dir, f"{namespace}_trajectory.yaml"),
                    'controller_topic': 'position_controller/joint_trajectory',
                    'use_sim_time': use_sim_time
                }],
                output='screen'
            )

            trajectory_publisher_cmd = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=position_controller_controller_spawner,
                    on_exit=[trajectory_publisher_node],
                )
            )

            actions = [obstacle_state_publisher, spawner_node, joint_state_cmd, drive_controller_cmd, trajectory_publisher_cmd]
        else:
            # For static obstacles, we process xacro manually to get the string
            # This avoids needing robot_state_publisher
            doc = xacro.process_file(temp_xacro_path, mappings={'namespace': namespace})
            robot_description_str = doc.toxml()
            
            spawner_node = Node(
                package='ros_gz_sim',
                executable='create',
                namespace=namespace,
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    '-name', f'{namespace}',
                    '-string', robot_description_str,
                    '-allow_renaming', 'true',
                    '-x', str(obstacle['x_pose']),
                    '-y', str(obstacle['y_pose']),
                    '-z', '0.5',
                ],
                output='screen',
            )
            actions = [spawner_node]

        delay_time = 5.0  # seconds
        delay_node = TimerAction(
            period=delay_time * obstacles.index(obstacle),
            actions=actions
        )
        ld.add_action(delay_node)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        )
    ld.add_action(clock_bridge)

    # Add IGN model path to env
    return ld
