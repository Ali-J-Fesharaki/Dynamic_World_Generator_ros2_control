#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import yaml
import os

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('controller_topic', 'position_controller/joint_trajectory')
        
        trajectory_file = self.get_parameter('trajectory_file').value
        topic_name = self.get_parameter('controller_topic').value
        
        if not trajectory_file:
            self.get_logger().error("No trajectory file specified")
            return

        if not os.path.exists(trajectory_file):
            self.get_logger().error(f"Trajectory file not found: {trajectory_file}")
            return

        self.publisher_ = self.create_publisher(JointTrajectory, topic_name, 10)
        
        # Load trajectory
        try:
            with open(trajectory_file, 'r') as f:
                data = yaml.safe_load(f)
                
            self.msg = JointTrajectory()
            self.msg.joint_names = data['joints']
            
            for p in data['points']:
                point = JointTrajectoryPoint()
                point.positions = [float(x) for x in p['positions']]
                
                # Convert time_from_start (float seconds) to Duration
                time_val = float(p['time_from_start'])
                secs = int(time_val)
                nsecs = int((time_val - secs) * 1e9)
                point.time_from_start = Duration(sec=secs, nanosec=nsecs)
                
                self.msg.points.append(point)
                
            self.get_logger().info(f"Loaded trajectory with {len(self.msg.points)} points for {data['obstacle_name']}")
            
            # Calculate duration
            self.duration = 0.0
            if self.msg.points:
                last_point = self.msg.points[-1]
                self.duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9

            # Publish once after a delay to ensure connection
            # We wait a bit to make sure the controller is up and subscriber is ready
            self.timer = self.create_timer(5.0, self.publish_trajectory)
            
        except Exception as e:
            self.get_logger().error(f"Failed to load trajectory: {str(e)}")

    def publish_trajectory(self):
        if self.publisher_.get_subscription_count() > 0:
            # FIX: Set timestamp to 0 to force immediate execution.
            # Using now() can cause the controller to wait indefinitely if 
            # there is a mismatch between Wall Time and Sim Time.
            self.msg.header.stamp.sec = 0
            self.msg.header.stamp.nanosec = 0
            
            self.publisher_.publish(self.msg)
            self.get_logger().info(f"Published trajectory to {self.publisher_.topic_name}")
            
            # Repeat trajectory
            if self.duration > 0:
                self.timer.cancel()
                self.timer = self.create_timer(self.duration + 0.5, self.publish_trajectory)
            else:
                self.timer.cancel()
        else:
            self.get_logger().warn(f"Waiting for subscribers on {self.publisher_.topic_name}...")

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
