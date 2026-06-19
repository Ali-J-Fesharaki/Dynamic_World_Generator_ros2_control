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
Trajectory Publisher for Isaac Sim dynamic obstacles.
Publishes trajectory commands for controlling obstacle motion via ROS2.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
import yaml
import os


class TrajectoryPublisher(Node):
    """
    ROS2 Node that publishes trajectory messages for Isaac Sim obstacle control.
    
    Supports two modes:
    1. Joint trajectory mode: Publishes JointTrajectory messages for ros2_control
    2. Pose mode: Publishes PoseStamped messages for direct position control
    """
    
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        # Declare parameters
        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('controller_topic', 'position_controller/joint_trajectory')
        self.declare_parameter('use_pose_mode', False)
        self.declare_parameter('pose_topic', 'target_pose')
        
        trajectory_file = self.get_parameter('trajectory_file').value
        topic_name = self.get_parameter('controller_topic').value
        self.use_pose_mode = self.get_parameter('use_pose_mode').value
        pose_topic = self.get_parameter('pose_topic').value
        
        if not trajectory_file:
            self.get_logger().error("No trajectory file specified")
            return
        
        if not os.path.exists(trajectory_file):
            self.get_logger().error(f"Trajectory file not found: {trajectory_file}")
            return
        
        # Create publishers based on mode
        if self.use_pose_mode:
            self.pose_publisher = self.create_publisher(PoseStamped, pose_topic, 10)
            self.trajectory_publisher = None
        else:
            self.trajectory_publisher = self.create_publisher(JointTrajectory, topic_name, 10)
            self.pose_publisher = None
        
        # Load trajectory
        try:
            with open(trajectory_file, 'r') as f:
                data = yaml.safe_load(f)
            
            self.obstacle_name = data.get('obstacle_name', 'unknown')
            self.points = data.get('points', [])
            self.joint_names = data.get('joints', ['joint_x', 'joint_y', 'joint_z'])
            
            if self.use_pose_mode:
                self._setup_pose_mode(data)
            else:
                self._setup_trajectory_mode(data)
            
            self.get_logger().info(
                f"Loaded trajectory with {len(self.points)} points for {self.obstacle_name}"
            )
            
            # Calculate total duration
            self.duration = 0.0
            if self.points:
                last_point = self.points[-1]
                self.duration = float(last_point.get('time_from_start', 0.0))
            
            # Start publishing after a delay to ensure connections
            self.current_point_index = 0
            self.timer = self.create_timer(5.0, self.publish_trajectory)
            
        except (yaml.YAMLError, KeyError, FileNotFoundError) as e:
            self.get_logger().error(f"Failed to load trajectory: {str(e)}")
    
    def _setup_trajectory_mode(self, data):
        """Setup for joint trajectory publishing mode."""
        self.msg = JointTrajectory()
        self.msg.joint_names = self.joint_names
        
        for p in self.points:
            point = JointTrajectoryPoint()
            point.positions = [float(x) for x in p['positions']]
            
            # Convert time_from_start to Duration
            time_val = float(p['time_from_start'])
            secs = int(time_val)
            nsecs = int((time_val - secs) * 1e9)
            point.time_from_start = Duration(sec=secs, nanosec=nsecs)
            
            self.msg.points.append(point)
    
    def _setup_pose_mode(self, data):
        """Setup for pose publishing mode."""
        self.pose_msgs = []
        
        for p in self.points:
            msg = PoseStamped()
            msg.header.frame_id = 'world'
            
            positions = p['positions']
            msg.pose.position.x = float(positions[0])
            msg.pose.position.y = float(positions[1])
            msg.pose.position.z = float(positions[2]) if len(positions) > 2 else 0.0
            
            # Default orientation (no rotation)
            msg.pose.orientation.w = 1.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            
            self.pose_msgs.append({
                'msg': msg,
                'time_from_start': float(p['time_from_start'])
            })
    
    def publish_trajectory(self):
        """Publish trajectory based on current mode."""
        if self.use_pose_mode:
            self._publish_pose()
        else:
            self._publish_joint_trajectory()
    
    def _publish_joint_trajectory(self):
        """Publish full joint trajectory message."""
        if self.trajectory_publisher is None:
            return
        
        if self.trajectory_publisher.get_subscription_count() > 0:
            # Set timestamp to 0 for immediate execution
            self.msg.header.stamp.sec = 0
            self.msg.header.stamp.nanosec = 0
            self.msg.header.frame_id = 'world'
            
            self.trajectory_publisher.publish(self.msg)
            self.get_logger().info(
                f"Published trajectory to {self.trajectory_publisher.topic_name}"
            )
            
            # Reschedule for trajectory repeat
            if self.duration > 0:
                self.timer.cancel()
                self.timer = self.create_timer(self.duration + 0.5, self.publish_trajectory)
            else:
                self.timer.cancel()
        else:
            self.get_logger().warn(
                f"Waiting for subscribers on {self.trajectory_publisher.topic_name}..."
            )
    
    def _publish_pose(self):
        """Publish individual pose messages sequentially."""
        if self.pose_publisher is None or not self.pose_msgs:
            return
        
        if self.pose_publisher.get_subscription_count() > 0:
            if self.current_point_index < len(self.pose_msgs):
                pose_data = self.pose_msgs[self.current_point_index]
                msg = pose_data['msg']
                
                # Update timestamp
                now = self.get_clock().now()
                msg.header.stamp = now.to_msg()
                
                self.pose_publisher.publish(msg)
                
                self.current_point_index += 1
                
                # Schedule next point
                if self.current_point_index < len(self.pose_msgs):
                    next_time = self.pose_msgs[self.current_point_index]['time_from_start']
                    current_time = pose_data['time_from_start']
                    delay = max(0.1, next_time - current_time)
                    
                    self.timer.cancel()
                    self.timer = self.create_timer(delay, self.publish_trajectory)
                else:
                    # Reset for loop
                    self.current_point_index = 0
                    self.timer.cancel()
                    self.timer = self.create_timer(1.0, self.publish_trajectory)
        else:
            self.get_logger().warn(
                f"Waiting for subscribers on {self.pose_publisher.topic_name}..."
            )


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
