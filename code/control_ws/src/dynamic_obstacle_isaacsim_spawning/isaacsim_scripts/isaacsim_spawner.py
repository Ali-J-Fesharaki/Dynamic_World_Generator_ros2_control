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
Isaac Sim Obstacle Spawner Node.

This script provides a ROS2 node interface for spawning and managing
dynamic obstacles in Isaac Sim. It can be used in conjunction with
the Isaac Sim ROS2 Bridge.

For standalone Isaac Sim usage (without ROS), see isaacsim_standalone.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml
import os
import json

from isaacsim_scripts.utils import (
    IsaacSimObstacleLoader,
    CircleMotionController,
    LinearMotionController,
    EllipticalMotionController,
    PolygonMotionController,
    NeuralNetworkMotionController,
)


class IsaacSimSpawnerNode(Node):
    """
    ROS2 Node for spawning and controlling obstacles in Isaac Sim.
    
    This node provides:
    - Loading obstacle configurations from YAML
    - Publishing spawn commands for Isaac Sim
    - Managing motion controllers for dynamic obstacles
    - Interface with Isaac Sim ROS2 Bridge
    """
    
    def __init__(self):
        super().__init__('isaacsim_spawner')
        
        # Declare parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('controllers_config', '')
        self.declare_parameter('spawn_script_output', '/tmp/isaacsim_spawn.py')
        self.declare_parameter('trajectory_output_dir', '/tmp')
        self.declare_parameter('update_rate', 30.0)
        
        config_file = self.get_parameter('config_file').value
        controllers_config = self.get_parameter('controllers_config').value
        self.spawn_script_output = self.get_parameter('spawn_script_output').value
        self.trajectory_output_dir = self.get_parameter('trajectory_output_dir').value
        update_rate = self.get_parameter('update_rate').value
        
        # Initialize obstacle loader
        self.obstacle_loader = None
        self.obstacles = []
        self.motion_controllers = {}
        self.obstacle_positions = {}
        
        if config_file and os.path.exists(config_file):
            self.obstacle_loader = IsaacSimObstacleLoader(
                config_path=config_file,
                controllers_config_path=controllers_config if controllers_config else None
            )
            self._load_obstacles()
        else:
            self.get_logger().warn(f"Config file not found or not specified: {config_file}")
        
        # Publishers
        self.spawn_command_pub = self.create_publisher(
            String, 'isaacsim/spawn_command', 10
        )
        
        # Create pose publishers for each dynamic obstacle
        self.pose_publishers = {}
        for obstacle in self.obstacles:
            if obstacle['has_motion']:
                topic = f"obstacle/{obstacle['name']}/target_pose"
                self.pose_publishers[obstacle['name']] = self.create_publisher(
                    PoseStamped, topic, 10
                )
        
        # Services
        self.spawn_all_srv = self.create_service(
            Trigger, 'spawn_all_obstacles', self.spawn_all_callback
        )
        self.generate_script_srv = self.create_service(
            Trigger, 'generate_spawn_script', self.generate_script_callback
        )
        
        # Timer for motion controller updates
        if update_rate > 0 and self.motion_controllers:
            period = 1.0 / update_rate
            self.update_timer = self.create_timer(period, self.update_motion)
            self.last_update_time = self.get_clock().now()
        else:
            self.update_timer = None
        
        self.get_logger().info(
            f"Isaac Sim Spawner initialized with {len(self.obstacles)} obstacles"
        )
    
    def _load_obstacles(self):
        """Load obstacles from configuration and setup motion controllers."""
        self.obstacles = self.obstacle_loader.get_obstacle_list()
        
        for obstacle in self.obstacles:
            name = obstacle['name']
            position = [obstacle['x_pose'], obstacle['y_pose'], obstacle['z_pose']]
            self.obstacle_positions[name] = position
            
            if obstacle['has_motion']:
                controller = self.obstacle_loader.create_motion_controller(
                    obstacle['motion'],
                    position
                )
                if controller:
                    self.motion_controllers[name] = controller
                    controller.reset(position)
        
        # Generate trajectory files for ROS2 control compatibility
        if self.trajectory_output_dir:
            self.obstacle_loader.generate_trajectory_yaml(self.trajectory_output_dir)
            self.get_logger().info(
                f"Generated trajectory files in {self.trajectory_output_dir}"
            )
    
    def update_motion(self):
        """Update motion controllers and publish new positions."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = current_time
        
        for name, controller in self.motion_controllers.items():
            if name in self.obstacle_positions:
                current_pos = self.obstacle_positions[name]
                new_pos = controller.update(dt, current_pos)
                self.obstacle_positions[name] = new_pos
                
                # Publish pose update
                if name in self.pose_publishers:
                    msg = PoseStamped()
                    msg.header.stamp = current_time.to_msg()
                    msg.header.frame_id = 'world'
                    msg.pose.position.x = new_pos[0]
                    msg.pose.position.y = new_pos[1]
                    msg.pose.position.z = new_pos[2]
                    msg.pose.orientation.w = 1.0
                    
                    self.pose_publishers[name].publish(msg)
    
    def spawn_all_callback(self, request, response):
        """Service callback to spawn all configured obstacles."""
        if self.obstacle_loader is None:
            response.success = False
            response.message = "No obstacle configuration loaded"
            return response
        
        try:
            # Generate and publish spawn commands
            for obstacle in self.obstacles:
                script = self.obstacle_loader.generate_spawn_script(obstacle)
                
                msg = String()
                msg.data = json.dumps({
                    'type': 'spawn',
                    'obstacle': obstacle,
                    'script': script
                })
                self.spawn_command_pub.publish(msg)
                
                self.get_logger().info(f"Published spawn command for {obstacle['name']}")
            
            response.success = True
            response.message = f"Spawned {len(self.obstacles)} obstacles"
        except (TypeError, ValueError, KeyError, AttributeError) as e:
            response.success = False
            response.message = f"Error spawning obstacles: {str(e)}"
            self.get_logger().error(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Unexpected error: {str(e)}"
            self.get_logger().error(response.message)
        
        return response
    
    def generate_script_callback(self, request, response):
        """Service callback to generate spawn script file."""
        if self.obstacle_loader is None:
            response.success = False
            response.message = "No obstacle configuration loaded"
            return response
        
        try:
            self.obstacle_loader.save_spawn_script_to_file(self.spawn_script_output)
            response.success = True
            response.message = f"Script saved to {self.spawn_script_output}"
            self.get_logger().info(response.message)
        except (OSError, IOError, PermissionError) as e:
            response.success = False
            response.message = f"File error: {str(e)}"
            self.get_logger().error(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Unexpected error: {str(e)}"
            self.get_logger().error(response.message)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = IsaacSimSpawnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
