#!/usr/bin/env python3
"""
| File: people.py
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: ROS2 Control integration for person/obstacle simulation in Isaac Sim.
| This module provides controllers that work with ros2_control to command Isaac Sim objects.
"""

import math
import numpy as np
from typing import List, Tuple, Optional, Dict, Any, Literal
from abc import ABC, abstractmethod


class PersonControllerBase(ABC):
    """
    Abstract base class for person controllers that integrate with ros2_control.
    
    Controllers generate trajectory points that are published via ros2_control's
    JointTrajectoryController to move persons/obstacles in Isaac Sim.
    """
    
    def __init__(self, name: str, init_pos: List[float] = None):
        """
        Initialize the person controller.
        
        Args:
            name: Unique name for this person/obstacle
            init_pos: Initial position [x, y, z], defaults to [0, 0, 0]
        """
        self.name = name
        self.init_pos = init_pos if init_pos else [0.0, 0.0, 0.0]
        self._current_time = 0.0
        
    @abstractmethod
    def get_trajectory_points(self, duration: float, dt: float = 0.5) -> List[Dict[str, Any]]:
        """
        Generate trajectory points for the controller.
        
        Args:
            duration: Total duration of the trajectory in seconds
            dt: Time step between points in seconds
            
        Returns:
            List of trajectory points with 'time_from_start' and 'positions' keys
        """
        pass
    
    @abstractmethod
    def update(self, dt: float) -> Tuple[float, float, float]:
        """
        Update the controller state and return the target position.
        
        Args:
            dt: Time step in seconds
            
        Returns:
            Tuple of (x, y, z) target position
        """
        pass


class CirclePersonController(PersonControllerBase):
    """
    Controller that moves a person in a circular path around a center point.
    
    This controller generates trajectory points compatible with ros2_control's
    JointTrajectoryController for controlling prismatic joints (joint_x, joint_y, joint_z).
    """
    
    def __init__(
        self,
        name: str,
        init_pos: List[float] = None,
        radius: float = 5.0,
        angular_velocity: float = 0.3,
        center: List[float] = None
    ):
        """
        Initialize circle person controller.
        
        Args:
            name: Unique name for this person/obstacle
            init_pos: Initial position [x, y, z]
            radius: Radius of the circular path in meters
            angular_velocity: Angular velocity in rad/s
            center: Center of circular motion [x, y], defaults to [0, 0]
        """
        super().__init__(name, init_pos)
        self.radius = radius
        self.angular_velocity = angular_velocity
        self.center = center if center else [0.0, 0.0]
        self.gamma = 0.0  # Current angle
        
        # Calculate initial angle based on initial position
        if init_pos:
            dx = init_pos[0] - self.center[0]
            dy = init_pos[1] - self.center[1]
            self.gamma = math.atan2(dy, dx)
    
    def update(self, dt: float) -> Tuple[float, float, float]:
        """
        Update controller and return target position.
        
        Args:
            dt: Time step in seconds
            
        Returns:
            Tuple of (x, y, z) target position
        """
        self.gamma += self.angular_velocity * dt
        self._current_time += dt
        
        x = self.center[0] + self.radius * math.cos(self.gamma)
        y = self.center[1] + self.radius * math.sin(self.gamma)
        z = self.init_pos[2] if self.init_pos else 0.0
        
        return (x, y, z)
    
    def get_trajectory_points(self, duration: float, dt: float = 0.5) -> List[Dict[str, Any]]:
        """
        Generate circular trajectory points for ros2_control.
        
        Args:
            duration: Total duration of the trajectory in seconds
            dt: Time step between points in seconds
            
        Returns:
            List of trajectory points compatible with JointTrajectory message
        """
        points = []
        current_time = 0.0
        gamma = self.gamma
        z = self.init_pos[2] if self.init_pos else 0.0
        
        while current_time <= duration:
            x = self.center[0] + self.radius * math.cos(gamma)
            y = self.center[1] + self.radius * math.sin(gamma)
            
            points.append({
                'time_from_start': current_time,
                'positions': [x, y, z]
            })
            
            gamma += self.angular_velocity * dt
            current_time += dt
        
        return points


class LinearPersonController(PersonControllerBase):
    """
    Controller that moves a person along a linear path between two points.
    
    This controller generates trajectory points compatible with ros2_control's
    JointTrajectoryController.
    """
    
    def __init__(
        self,
        name: str,
        init_pos: List[float] = None,
        end_pos: List[float] = None,
        velocity: float = 1.0,
        oscillate: bool = True
    ):
        """
        Initialize linear person controller.
        
        Args:
            name: Unique name for this person/obstacle
            init_pos: Start position [x, y, z]
            end_pos: End position [x, y, z]
            velocity: Movement velocity in m/s
            oscillate: If True, oscillate back and forth; if False, stop at end
        """
        super().__init__(name, init_pos)
        self.end_pos = end_pos if end_pos else [5.0, 0.0, 0.0]
        self.velocity = velocity
        self.oscillate = oscillate
        self._progress = 0.0
        self._direction = 1
        
        # Calculate path distance
        dx = self.end_pos[0] - self.init_pos[0]
        dy = self.end_pos[1] - self.init_pos[1]
        dz = self.end_pos[2] - self.init_pos[2]
        self._distance = math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def update(self, dt: float) -> Tuple[float, float, float]:
        """
        Update controller and return target position.
        
        Args:
            dt: Time step in seconds
            
        Returns:
            Tuple of (x, y, z) target position
        """
        self._current_time += dt
        
        # Update progress
        if self._distance > 0:
            delta = (self.velocity * dt) / self._distance
            self._progress += delta * self._direction
            
            if self._progress >= 1.0:
                if self.oscillate:
                    self._progress = 1.0
                    self._direction = -1
                else:
                    self._progress = 1.0
            elif self._progress <= 0.0:
                self._progress = 0.0
                self._direction = 1
        
        # Interpolate position
        x = self.init_pos[0] + self._progress * (self.end_pos[0] - self.init_pos[0])
        y = self.init_pos[1] + self._progress * (self.end_pos[1] - self.init_pos[1])
        z = self.init_pos[2] + self._progress * (self.end_pos[2] - self.init_pos[2])
        
        return (x, y, z)
    
    def get_trajectory_points(self, duration: float, dt: float = 0.5) -> List[Dict[str, Any]]:
        """
        Generate linear trajectory points for ros2_control.
        
        Args:
            duration: Total duration of the trajectory in seconds
            dt: Time step between points in seconds
            
        Returns:
            List of trajectory points compatible with JointTrajectory message
        """
        points = []
        current_time = 0.0
        progress = 0.0
        direction = 1
        
        while current_time <= duration:
            x = self.init_pos[0] + progress * (self.end_pos[0] - self.init_pos[0])
            y = self.init_pos[1] + progress * (self.end_pos[1] - self.init_pos[1])
            z = self.init_pos[2] + progress * (self.end_pos[2] - self.init_pos[2])
            
            points.append({
                'time_from_start': current_time,
                'positions': [x, y, z]
            })
            
            # Update progress
            if self._distance > 0:
                delta = (self.velocity * dt) / self._distance
                progress += delta * direction
                
                if progress >= 1.0:
                    if self.oscillate:
                        progress = 1.0
                        direction = -1
                    else:
                        progress = 1.0
                elif progress <= 0.0:
                    progress = 0.0
                    direction = 1
            
            current_time += dt
        
        return points


class PolygonPersonController(PersonControllerBase):
    """
    Controller that moves a person along a polygon path defined by waypoints.
    
    This controller generates trajectory points compatible with ros2_control's
    JointTrajectoryController.
    """
    
    def __init__(
        self,
        name: str,
        init_pos: List[float] = None,
        waypoints: List[List[float]] = None,
        velocity: float = 1.0,
        loop: bool = True
    ):
        """
        Initialize polygon person controller.
        
        Args:
            name: Unique name for this person/obstacle
            init_pos: Initial position [x, y, z] (overridden by first waypoint if provided)
            waypoints: List of waypoints [[x1, y1], [x2, y2], ...]
            velocity: Movement velocity in m/s
            loop: If True, loop back to start; if False, stop at end
        """
        super().__init__(name, init_pos)
        self.waypoints = waypoints if waypoints else [[0, 0], [5, 0], [5, 5], [0, 5]]
        self.velocity = velocity
        self.loop = loop
        self._current_waypoint_idx = 0
        self._segment_progress = 0.0
        self._z = init_pos[2] if init_pos else 0.0
    
    def _get_segment_distance(self, idx: int) -> float:
        """Calculate distance between waypoint idx and idx+1."""
        if idx >= len(self.waypoints) - 1:
            if self.loop:
                p1 = self.waypoints[-1]
                p2 = self.waypoints[0]
            else:
                return 0.0
        else:
            p1 = self.waypoints[idx]
            p2 = self.waypoints[idx + 1]
        
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.sqrt(dx*dx + dy*dy)
    
    def update(self, dt: float) -> Tuple[float, float, float]:
        """
        Update controller and return target position.
        
        Args:
            dt: Time step in seconds
            
        Returns:
            Tuple of (x, y, z) target position
        """
        self._current_time += dt
        
        segment_dist = self._get_segment_distance(self._current_waypoint_idx)
        
        if segment_dist > 0:
            delta = (self.velocity * dt) / segment_dist
            self._segment_progress += delta
            
            while self._segment_progress >= 1.0:
                self._segment_progress -= 1.0
                self._current_waypoint_idx += 1
                
                if self._current_waypoint_idx >= len(self.waypoints):
                    if self.loop:
                        self._current_waypoint_idx = 0
                    else:
                        self._current_waypoint_idx = len(self.waypoints) - 1
                        self._segment_progress = 0.0
                        break
                
                segment_dist = self._get_segment_distance(self._current_waypoint_idx)
                if segment_dist == 0:
                    break
        
        # Interpolate position
        p1 = self.waypoints[self._current_waypoint_idx]
        if self._current_waypoint_idx < len(self.waypoints) - 1:
            p2 = self.waypoints[self._current_waypoint_idx + 1]
        elif self.loop:
            p2 = self.waypoints[0]
        else:
            p2 = p1
        
        x = p1[0] + self._segment_progress * (p2[0] - p1[0])
        y = p1[1] + self._segment_progress * (p2[1] - p1[1])
        
        return (x, y, self._z)
    
    def get_trajectory_points(self, duration: float, dt: float = 0.5) -> List[Dict[str, Any]]:
        """
        Generate polygon trajectory points for ros2_control.
        
        Args:
            duration: Total duration of the trajectory in seconds
            dt: Time step between points in seconds
            
        Returns:
            List of trajectory points compatible with JointTrajectory message
        """
        points = []
        current_time = 0.0
        waypoint_idx = 0
        segment_progress = 0.0
        
        while current_time <= duration:
            p1 = self.waypoints[waypoint_idx]
            if waypoint_idx < len(self.waypoints) - 1:
                p2 = self.waypoints[waypoint_idx + 1]
            elif self.loop:
                p2 = self.waypoints[0]
            else:
                p2 = p1
            
            x = p1[0] + segment_progress * (p2[0] - p1[0])
            y = p1[1] + segment_progress * (p2[1] - p1[1])
            
            points.append({
                'time_from_start': current_time,
                'positions': [x, y, self._z]
            })
            
            # Update progress
            segment_dist = self._get_segment_distance(waypoint_idx)
            if segment_dist > 0:
                delta = (self.velocity * dt) / segment_dist
                segment_progress += delta
                
                while segment_progress >= 1.0:
                    segment_progress -= 1.0
                    waypoint_idx += 1
                    
                    if waypoint_idx >= len(self.waypoints):
                        if self.loop:
                            waypoint_idx = 0
                        else:
                            waypoint_idx = len(self.waypoints) - 1
                            segment_progress = 0.0
                            break
                    
                    segment_dist = self._get_segment_distance(waypoint_idx)
                    if segment_dist == 0:
                        break
            
            current_time += dt
        
        return points


def generate_ros2_control_xacro_isaacsim(
    name: str,
    controllers_config_path: str
) -> str:
    """
    Generate ros2_control XACRO configuration for Isaac Sim.
    
    This generates the ros2_control hardware interface configuration that works
    with Isaac Sim's ros2_control integration, similar to gz_ros2_control for Gazebo.
    
    Args:
        name: Name of the obstacle/person for namespacing
        controllers_config_path: Path to the ros2_controllers.yaml file (used for reference)
        
    Returns:
        XML string containing the ros2_control configuration
    """
    return f'''    <ros2_control name="{name}" type="system">
      <hardware>
        <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
        <param name="joint_commands_topic">/{name}/joint_commands</param>
        <param name="joint_states_topic">/{name}/joint_states</param>
        <param name="controllers_config">{controllers_config_path}</param>
      </hardware>
      <joint name="joint_x">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
      <joint name="joint_y">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
      <joint name="joint_z">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
    </ros2_control>'''


def generate_person_urdf_isaacsim(
    name: str,
    person_type: str = "box",
    size: Tuple[float, ...] = (0.5, 0.5, 1.8),
    color: str = "blue",
    controllers_config_path: Optional[str] = None
) -> str:
    """
    Generate a URDF for a person/obstacle that can be loaded in Isaac Sim with ros2_control.
    
    Args:
        name: Unique name for the person/obstacle
        person_type: Type of geometry ('box', 'cylinder', 'sphere')
        size: Size parameters (box: w,l,h; cylinder: r,h; sphere: r)
        color: Color name for visualization
        controllers_config_path: Path to ros2_controllers.yaml
        
    Returns:
        URDF XML string
    """
    # Color mapping
    colors = {
        'red': (1.0, 0.0, 0.0),
        'green': (0.0, 1.0, 0.0),
        'blue': (0.0, 0.0, 1.0),
        'yellow': (1.0, 1.0, 0.0),
        'gray': (0.5, 0.5, 0.5),
        'white': (1.0, 1.0, 1.0),
        'black': (0.1, 0.1, 0.1),
    }
    rgb = colors.get(color.lower(), (0.5, 0.5, 0.5))
    
    # Generate geometry
    if person_type == "box":
        geometry = f'<box size="{size[0]} {size[1]} {size[2]}"/>'
    elif person_type == "cylinder":
        geometry = f'<cylinder radius="{size[0]}" length="{size[1]}"/>'
    elif person_type == "sphere":
        geometry = f'<sphere radius="{size[0]}"/>'
    else:
        geometry = f'<box size="0.5 0.5 1.8"/>'
    
    urdf = f'''<?xml version="1.0"?>
<robot name="{name}" xmlns:xacro="http://www.ros.org/wiki/xacro">
    
    <link name="world"/>
    
    <!-- Prismatic joint chain for XYZ motion -->
    <link name="{name}/link_x">
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.001"/>
        <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
      </inertial>
    </link>
    
    <link name="{name}/link_y">
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.001"/>
        <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
      </inertial>
    </link>
    
    <link name="{name}/link_z">
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.001"/>
        <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
      </inertial>
    </link>

    <joint name="joint_x" type="prismatic">
      <parent link="world"/>
      <child link="{name}/link_x"/>
      <axis xyz="1 0 0"/>
      <limit lower="-1000" upper="1000" effort="1000.0" velocity="100.0"/>
      <dynamics damping="0.0" friction="0.0"/>
    </joint>

    <joint name="joint_y" type="prismatic">
      <parent link="{name}/link_x"/>
      <child link="{name}/link_y"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1000" upper="1000" effort="1000.0" velocity="100.0"/>
      <dynamics damping="0.0" friction="0.0"/>
    </joint>

    <joint name="joint_z" type="prismatic">
      <parent link="{name}/link_y"/>
      <child link="{name}/link_z"/>
      <axis xyz="0 0 1"/>
      <limit lower="-1000" upper="1000" effort="1000.0" velocity="100.0"/>
      <dynamics damping="0.0" friction="0.0"/>
    </joint>
    
    <!-- Main visual/collision link -->
    <link name="{name}/link">
      <visual>
        <geometry>
          {geometry}
        </geometry>
        <material name="{color}">
          <color rgba="{rgb[0]} {rgb[1]} {rgb[2]} 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          {geometry}
        </geometry>
      </collision>
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="1.0"/>
        <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
      </inertial>
    </link>
    
    <joint name="fixed_joint" type="fixed">
      <parent link="{name}/link_z"/>
      <child link="{name}/link"/>
    </joint>
'''
    
    # Add ros2_control configuration if controllers_config_path is provided
    if controllers_config_path:
        urdf += f'''
    {generate_ros2_control_xacro_isaacsim(name, controllers_config_path)}
'''
    
    urdf += '''
</robot>'''
    
    return urdf


def generate_trajectory_yaml(
    controller: PersonControllerBase,
    duration: float = 60.0,
    dt: float = 0.5
) -> Dict[str, Any]:
    """
    Generate trajectory YAML data from a controller for ros2_control.
    
    Args:
        controller: Person controller instance
        duration: Total trajectory duration in seconds
        dt: Time step between points
        
    Returns:
        Dictionary with trajectory data compatible with trajectory_publisher
    """
    points = controller.get_trajectory_points(duration, dt)
    
    return {
        'obstacle_name': controller.name,
        'joints': ['joint_x', 'joint_y', 'joint_z'],
        'points': points
    }


# Example usage and testing
if __name__ == "__main__":
    import yaml
    
    # Create a circle controller
    circle_ctrl = CirclePersonController(
        name="person1",
        init_pos=[5.0, 0.0, 0.9],
        radius=5.0,
        angular_velocity=0.3
    )
    
    # Create a linear controller
    linear_ctrl = LinearPersonController(
        name="person2",
        init_pos=[0.0, 0.0, 0.9],
        end_pos=[10.0, 0.0, 0.9],
        velocity=1.0,
        oscillate=True
    )
    
    # Create a polygon controller
    polygon_ctrl = PolygonPersonController(
        name="person3",
        init_pos=[0.0, 0.0, 0.9],
        waypoints=[[0, 0], [5, 0], [5, 5], [0, 5]],
        velocity=1.0,
        loop=True
    )
    
    # Generate trajectory YAML for each controller
    for ctrl in [circle_ctrl, linear_ctrl, polygon_ctrl]:
        trajectory_data = generate_trajectory_yaml(ctrl, duration=30.0, dt=0.5)
        print(f"\n=== Trajectory for {ctrl.name} ===")
        print(f"Joints: {trajectory_data['joints']}")
        print(f"Number of points: {len(trajectory_data['points'])}")
        print(f"First 3 points: {trajectory_data['points'][:3]}")
    
    # Generate URDF for Isaac Sim
    urdf = generate_person_urdf_isaacsim(
        name="person1",
        person_type="box",
        size=(0.5, 0.5, 1.8),
        color="blue",
        controllers_config_path="/path/to/controllers.yaml"
    )
    print("\n=== Generated URDF (truncated) ===")
    print(urdf[:500] + "...")
