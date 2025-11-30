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
Utility functions for Isaac Sim dynamic obstacle spawning.
This module provides:
- USD model generation for obstacles (box, cylinder, sphere)
- Motion controller base classes and implementations
- Person/character model support with configurable motion patterns
- Obstacle configuration loading from YAML files
"""

import os
import yaml
import math
import random
import re
from typing import Literal, Tuple, Optional, Dict, Any, List
from abc import ABC, abstractmethod


def sanitize_class_name(name: str) -> str:
    """
    Sanitize a string to be a valid Python class name.
    
    Args:
        name: Input string
    
    Returns:
        Valid Python class name
    """
    # Remove invalid characters and replace with underscores
    sanitized = re.sub(r'[^a-zA-Z0-9_]', '_', name)
    # Convert to title case and remove underscores for class name
    parts = sanitized.split('_')
    class_name = ''.join(part.capitalize() for part in parts if part)
    # Ensure it starts with a letter (add prefix if it starts with digit)
    if class_name and class_name[0].isdigit():
        class_name = 'Obs' + class_name
    # Ensure we have something
    return class_name if class_name else 'DefaultClass'


# ============================================================================
# Motion Controller Base Classes
# ============================================================================

class MotionController(ABC):
    """
    Abstract base class for motion controllers.
    Motion controllers define how obstacles move in the simulation.
    """
    
    def __init__(self):
        self._target = None
        self._velocity = 1.0
    
    @abstractmethod
    def update(self, dt: float, current_position: List[float]) -> List[float]:
        """
        Update the controller and return the new target position.
        
        Args:
            dt: Time step in seconds
            current_position: Current position [x, y, z]
        
        Returns:
            New target position [x, y, z]
        """
        pass
    
    @abstractmethod
    def reset(self, initial_position: List[float]):
        """Reset the controller to its initial state."""
        pass


class CircleMotionController(MotionController):
    """
    Makes an obstacle move in a circle around a center point.
    Inspired by Pegasus Simulator's CirclePersonController.
    """
    
    def __init__(
        self,
        center: List[float] = None,
        radius: float = 5.0,
        angular_velocity: float = 0.3,
        z_height: float = 0.0
    ):
        super().__init__()
        self._center = center or [0.0, 0.0]
        self._radius = radius
        self._angular_velocity = angular_velocity
        self._z_height = z_height
        self._gamma = 0.0
    
    def update(self, dt: float, current_position: List[float]) -> List[float]:
        self._gamma += self._angular_velocity * dt
        
        x = self._center[0] + self._radius * math.cos(self._gamma)
        y = self._center[1] + self._radius * math.sin(self._gamma)
        
        return [x, y, self._z_height]
    
    def reset(self, initial_position: List[float]):
        self._gamma = 0.0
        if len(initial_position) >= 2:
            # Calculate initial gamma from position relative to center
            dx = initial_position[0] - self._center[0]
            dy = initial_position[1] - self._center[1]
            self._gamma = math.atan2(dy, dx)


class LinearMotionController(MotionController):
    """
    Makes an obstacle move along a linear path between two points.
    """
    
    def __init__(
        self,
        start_point: List[float],
        end_point: List[float],
        velocity: float = 1.0,
        loop: bool = True,
        z_height: float = 0.0
    ):
        super().__init__()
        self._start = start_point[:2] + [z_height]
        self._end = end_point[:2] + [z_height]
        self._velocity = velocity
        self._loop = loop
        self._z_height = z_height
        self._progress = 0.0
        self._direction = 1.0
        
        dx = self._end[0] - self._start[0]
        dy = self._end[1] - self._start[1]
        self._total_distance = math.sqrt(dx * dx + dy * dy)
    
    def update(self, dt: float, current_position: List[float]) -> List[float]:
        if self._total_distance < 1e-6:
            return self._start.copy()
        
        self._progress += (self._velocity * dt * self._direction) / self._total_distance
        
        if self._loop:
            if self._progress >= 1.0:
                self._progress = 1.0
                self._direction = -1.0
            elif self._progress <= 0.0:
                self._progress = 0.0
                self._direction = 1.0
        else:
            self._progress = max(0.0, min(1.0, self._progress))
        
        x = self._start[0] + self._progress * (self._end[0] - self._start[0])
        y = self._start[1] + self._progress * (self._end[1] - self._start[1])
        
        return [x, y, self._z_height]
    
    def reset(self, initial_position: List[float]):
        self._progress = 0.0
        self._direction = 1.0


class EllipticalMotionController(MotionController):
    """
    Makes an obstacle move along an elliptical path.
    """
    
    def __init__(
        self,
        center: List[float],
        semi_major: float,
        semi_minor: float,
        angle: float = 0.0,
        angular_velocity: float = 0.3,
        z_height: float = 0.0
    ):
        super().__init__()
        self._center = center[:2]
        self._semi_major = semi_major
        self._semi_minor = semi_minor
        self._angle = angle  # Rotation angle of ellipse
        self._angular_velocity = angular_velocity
        self._z_height = z_height
        self._gamma = 0.0
    
    def update(self, dt: float, current_position: List[float]) -> List[float]:
        self._gamma += self._angular_velocity * dt
        
        # Ellipse parametric equations (local frame)
        local_x = self._semi_major * math.cos(self._gamma)
        local_y = self._semi_minor * math.sin(self._gamma)
        
        # Rotate by ellipse angle and translate to center
        cos_a = math.cos(self._angle)
        sin_a = math.sin(self._angle)
        x = self._center[0] + local_x * cos_a - local_y * sin_a
        y = self._center[1] + local_x * sin_a + local_y * cos_a
        
        return [x, y, self._z_height]
    
    def reset(self, initial_position: List[float]):
        self._gamma = 0.0


class PolygonMotionController(MotionController):
    """
    Makes an obstacle move along a polygon path defined by multiple points.
    """
    
    def __init__(
        self,
        path_points: List[List[float]],
        velocity: float = 1.0,
        loop: bool = True,
        z_height: float = 0.0
    ):
        super().__init__()
        self._path = [[p[0], p[1], z_height] for p in path_points]
        self._velocity = velocity
        self._loop = loop
        self._z_height = z_height
        self._current_segment = 0
        self._segment_progress = 0.0
    
    def update(self, dt: float, current_position: List[float]) -> List[float]:
        if len(self._path) < 2:
            return self._path[0] if self._path else [0.0, 0.0, self._z_height]
        
        # Get current segment endpoints
        p1 = self._path[self._current_segment]
        next_idx = (self._current_segment + 1) % len(self._path)
        p2 = self._path[next_idx]
        
        # Calculate segment distance
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        segment_distance = math.sqrt(dx * dx + dy * dy)
        
        if segment_distance < 1e-6:
            self._current_segment = next_idx
            return p1.copy()
        
        # Update progress
        self._segment_progress += (self._velocity * dt) / segment_distance
        
        if self._segment_progress >= 1.0:
            self._segment_progress = 0.0
            self._current_segment = next_idx
            
            if not self._loop and self._current_segment == 0:
                return self._path[-1].copy()
        
        # Interpolate position
        x = p1[0] + self._segment_progress * dx
        y = p1[1] + self._segment_progress * dy
        
        return [x, y, self._z_height]
    
    def reset(self, initial_position: List[float]):
        self._current_segment = 0
        self._segment_progress = 0.0


class NeuralNetworkMotionController(MotionController):
    """
    Motion controller that uses a neural network model for motion prediction.
    This is a placeholder/interface for future neural network integration.
    
    The neural network can be trained to generate realistic motion patterns
    based on environment variables from Isaac Sim.
    """
    
    def __init__(
        self,
        model_path: Optional[str] = None,
        z_height: float = 0.0
    ):
        super().__init__()
        self._model_path = model_path
        self._z_height = z_height
        self._model = None
        self._env_variables = {}
        
        if model_path and os.path.exists(model_path):
            self._load_model(model_path)
    
    def _load_model(self, model_path: str):
        """
        Load a neural network model for motion prediction.
        Supports common formats: .pt (PyTorch), .onnx, .h5 (Keras)
        """
        # Placeholder for model loading
        # In actual implementation, load appropriate model based on extension
        pass
    
    def set_environment_variables(self, env_vars: Dict[str, Any]):
        """
        Set Isaac Sim environment variables for motion prediction.
        
        Args:
            env_vars: Dictionary of environment variables such as:
                - nearby_obstacles: List of nearby obstacle positions
                - target_position: Target position if available
                - velocity: Current velocity
                - etc.
        """
        self._env_variables = env_vars
    
    def update(self, dt: float, current_position: List[float]) -> List[float]:
        """
        Predict next position using the neural network.
        Falls back to staying in place if no model is loaded.
        """
        if self._model is None:
            # No model loaded - stay in current position
            return current_position.copy()
        
        # Placeholder for actual neural network inference
        # In actual implementation:
        # 1. Prepare input features from current_position and env_variables
        # 2. Run inference
        # 3. Return predicted position
        
        return current_position.copy()
    
    def reset(self, initial_position: List[float]):
        self._env_variables = {}


# ============================================================================
# USD Model Generation Utilities
# ============================================================================

def get_color_rgba(color_name: str) -> Tuple[float, float, float, float]:
    """
    Convert color name to RGBA values for USD materials.
    
    Args:
        color_name: Name of the color (e.g., 'red', 'blue', 'green')
    
    Returns:
        Tuple of RGBA values (0-1 range)
    """
    colors = {
        'red': (1.0, 0.0, 0.0, 1.0),
        'green': (0.0, 1.0, 0.0, 1.0),
        'blue': (0.0, 0.0, 1.0, 1.0),
        'yellow': (1.0, 1.0, 0.0, 1.0),
        'cyan': (0.0, 1.0, 1.0, 1.0),
        'magenta': (1.0, 0.0, 1.0, 1.0),
        'white': (1.0, 1.0, 1.0, 1.0),
        'black': (0.0, 0.0, 0.0, 1.0),
        'gray': (0.5, 0.5, 0.5, 1.0),
        'orange': (1.0, 0.5, 0.0, 1.0),
        'purple': (0.5, 0.0, 0.5, 1.0),
    }
    return colors.get(color_name.lower(), (0.5, 0.5, 0.5, 1.0))


def calculate_inertia(
    obstacle_type: str,
    size: Tuple[float, ...],
    density: float = 1.0
) -> Tuple[float, float, float, float]:
    """
    Calculate mass and inertia tensor for different obstacle types.
    
    Args:
        obstacle_type: Type of obstacle ('box', 'cylinder', 'sphere')
        size: Size parameters (depends on obstacle_type)
        density: Material density in kg/m³
    
    Returns:
        Tuple of (mass, ixx, iyy, izz)
    """
    if obstacle_type == "box":
        w, l, h = size
        mass = density * w * l * h
        ixx = mass / 12.0 * (l**2 + h**2)
        iyy = mass / 12.0 * (w**2 + h**2)
        izz = mass / 12.0 * (w**2 + l**2)
    elif obstacle_type == "cylinder":
        r, h = size
        mass = density * math.pi * r**2 * h
        ixx = mass / 12.0 * (3 * r**2 + h**2)
        iyy = ixx
        izz = mass / 2.0 * r**2
    elif obstacle_type == "sphere":
        r = size[0]
        mass = density * (4/3) * math.pi * r**3
        ixx = (2/5) * mass * r**2
        iyy = ixx
        izz = ixx
    else:
        mass = 1.0
        ixx = iyy = izz = 0.1
    
    return mass, ixx, iyy, izz


def generate_usd_python_script(
    name: str,
    obstacle_type: str,
    x_pose: float,
    y_pose: float,
    z_pose: float,
    size: Tuple[float, ...],
    color: str = "gray",
    has_motion: bool = False
) -> str:
    """
    Generate Python script code that creates a USD prim in Isaac Sim.
    
    This generates Python code that can be executed in Isaac Sim to create
    the obstacle as a USD prim.
    
    Args:
        name: Unique name for the obstacle
        obstacle_type: Type of obstacle ('box', 'cylinder', 'sphere')
        x_pose: X position in world frame
        y_pose: Y position in world frame
        z_pose: Z position in world frame
        size: Size parameters
        color: Color name
        has_motion: Whether obstacle is dynamic
    
    Returns:
        Python script code as string
    """
    color_rgba = get_color_rgba(color)
    prim_path = f"/World/{name}"
    
    if obstacle_type == "box":
        w, l, h = size
        script = f'''
# Create box obstacle: {name}
from pxr import Gf, UsdGeom, UsdPhysics
from isaacsim.core.utils.prims import create_prim

# Create the box prim
prim = create_prim(
    prim_path="{prim_path}",
    prim_type="Cube",
    position=Gf.Vec3d({x_pose}, {y_pose}, {z_pose}),
    scale=Gf.Vec3d({w}, {l}, {h})
)

# Set visual properties
geom = UsdGeom.Gprim(prim)
geom.CreateDisplayColorAttr(value=[({color_rgba[0]}, {color_rgba[1]}, {color_rgba[2]})])

# Add physics for collision
UsdPhysics.CollisionAPI.Apply(prim)
{f"UsdPhysics.RigidBodyAPI.Apply(prim)  # Dynamic obstacle" if has_motion else "# Static obstacle - no rigid body"}
'''
    elif obstacle_type == "cylinder":
        r, h = size
        script = f'''
# Create cylinder obstacle: {name}
from pxr import Gf, UsdGeom, UsdPhysics
from isaacsim.core.utils.prims import create_prim

# Create the cylinder prim
prim = create_prim(
    prim_path="{prim_path}",
    prim_type="Cylinder",
    position=Gf.Vec3d({x_pose}, {y_pose}, {z_pose}),
    attributes={{"radius": {r}, "height": {h}}}
)

# Set visual properties
geom = UsdGeom.Gprim(prim)
geom.CreateDisplayColorAttr(value=[({color_rgba[0]}, {color_rgba[1]}, {color_rgba[2]})])

# Add physics for collision
UsdPhysics.CollisionAPI.Apply(prim)
{f"UsdPhysics.RigidBodyAPI.Apply(prim)  # Dynamic obstacle" if has_motion else "# Static obstacle - no rigid body"}
'''
    elif obstacle_type == "sphere":
        r = size[0]
        script = f'''
# Create sphere obstacle: {name}
from pxr import Gf, UsdGeom, UsdPhysics
from isaacsim.core.utils.prims import create_prim

# Create the sphere prim
prim = create_prim(
    prim_path="{prim_path}",
    prim_type="Sphere",
    position=Gf.Vec3d({x_pose}, {y_pose}, {z_pose}),
    attributes={{"radius": {r}}}
)

# Set visual properties
geom = UsdGeom.Gprim(prim)
geom.CreateDisplayColorAttr(value=[({color_rgba[0]}, {color_rgba[1]}, {color_rgba[2]})])

# Add physics for collision
UsdPhysics.CollisionAPI.Apply(prim)
{f"UsdPhysics.RigidBodyAPI.Apply(prim)  # Dynamic obstacle" if has_motion else "# Static obstacle - no rigid body"}
'''
    else:
        script = f'# Unknown obstacle type: {obstacle_type}'
    
    return script


# ============================================================================
# Character/Person Assets
# ============================================================================

# List of available character assets in Isaac Sim
# These map to the Pegasus Simulator's Person class character assets
PERSON_ASSETS = {
    "male_adult_construction": "original_male_adult_construction_05",
    "female_adult_business": "original_female_adult_business_02",
    "male_adult_business": "original_male_adult_business_01",
    "female_adult_casual": "original_female_adult_casual_01",
    "male_adult_casual": "original_male_adult_casual_01",
}


def get_available_person_assets() -> List[str]:
    """
    Get list of available person character assets.
    
    Returns:
        List of character asset names
    """
    return list(PERSON_ASSETS.keys())


def generate_person_spawn_script(
    name: str,
    character_type: str,
    x_pose: float,
    y_pose: float,
    z_pose: float = 0.0,
    yaw: float = 0.0,
    controller_type: str = "circle"
) -> str:
    """
    Generate Python script code to spawn a person character in Isaac Sim.
    
    Based on Pegasus Simulator's Person class usage.
    
    Args:
        name: Unique name for the person
        character_type: Character asset type (e.g., 'male_adult_construction')
        x_pose: X position
        y_pose: Y position
        z_pose: Z position
        yaw: Initial yaw orientation in radians
        controller_type: Motion controller type ('circle', 'linear', 'target')
    
    Returns:
        Python script code as string
    """
    asset_name = PERSON_ASSETS.get(character_type, character_type)
    # Sanitize name for use as class name
    class_name = sanitize_class_name(name)
    
    script = f'''
# Spawn person: {name}
# Requires Pegasus Simulator or equivalent person extension
try:
    from pegasus.simulator.logic.people.person import Person
    from pegasus.simulator.logic.people.person_controller import PersonController
    
    class {class_name}Controller(PersonController):
        """Custom controller for {name}"""
        
        def __init__(self):
            super().__init__()
            self._radius = 5.0
            self.gamma = 0.0
            self.gamma_dot = 0.3
        
        def update(self, dt: float):
            import numpy as np
            self.gamma += self.gamma_dot * dt
            self._person.update_target_position([
                self._radius * np.cos(self.gamma),
                self._radius * np.sin(self.gamma),
                0.0
            ])
    
    # Create the person with controller
    controller = {class_name}Controller()
    person = Person(
        "{name}",
        "{asset_name}",
        init_pos=[{x_pose}, {y_pose}, {z_pose}],
        init_yaw={yaw},
        controller=controller
    )
except ImportError:
    print("Pegasus Simulator not available - person spawning requires Pegasus extension")
'''
    
    return script


# ============================================================================
# Isaac Sim Obstacle Loader
# ============================================================================

class IsaacSimObstacleLoader:
    """
    Loads obstacle configurations from YAML and generates Isaac Sim spawn scripts.
    Similar to ObstacleLoader for Gazebo but adapted for USD/Isaac Sim.
    """
    
    def __init__(
        self,
        config_path: str,
        controllers_config_path: Optional[str] = None
    ):
        """
        Initialize the IsaacSimObstacleLoader.
        
        Args:
            config_path: Path to obstacles.yaml configuration file
            controllers_config_path: Path to controllers config (for ROS2 control)
        """
        self.config_path = config_path
        self.controllers_config_path = controllers_config_path
        self.obstacles = []
    
    def load_config(self) -> List[Dict[str, Any]]:
        """
        Load obstacle configurations from YAML file.
        
        Returns:
            List of obstacle configuration dictionaries
        """
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Config file not found: {self.config_path}")
        
        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        if 'obstacles' not in config:
            raise ValueError("Config file must contain 'obstacles' key")
        
        self.obstacles = config['obstacles']
        return self.obstacles
    
    def parse_obstacle(self, obstacle_config: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Parse obstacle configuration into standardized format.
        
        Args:
            obstacle_config: Raw obstacle configuration from YAML
        
        Returns:
            Parsed obstacle parameters or None if disabled
        """
        name = obstacle_config['name']
        obstacle_type = obstacle_config['type']
        enabled = obstacle_config.get('enabled', True)
        
        if not enabled:
            return None
        
        # Extract position
        x_pose = obstacle_config['x_pose']
        y_pose = obstacle_config['y_pose']
        z_pose = obstacle_config['z_pose']
        
        # Extract size based on type
        size_list = obstacle_config['size']
        if obstacle_type == 'box':
            size = tuple(size_list[:3])
        elif obstacle_type == 'cylinder':
            size = tuple(size_list[:2])
        elif obstacle_type == 'sphere':
            size = tuple(size_list[:1])
        elif obstacle_type == 'person':
            size = ()  # Persons don't have configurable size
        else:
            raise ValueError(f"Unknown obstacle type: {obstacle_type}")
        
        # Extract color (optional)
        color = obstacle_config.get('color', 'gray')
        
        # Extract character type for persons
        character_type = obstacle_config.get('character_type', 'male_adult_construction')
        
        # Check if obstacle has motion
        has_motion = 'motion' in obstacle_config
        motion_config = obstacle_config.get('motion', None)
        
        return {
            'name': name,
            'type': obstacle_type,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'size': size,
            'color': color,
            'character_type': character_type,
            'has_motion': has_motion,
            'motion': motion_config
        }
    
    def get_obstacle_list(self) -> List[Dict[str, Any]]:
        """
        Get list of parsed obstacles with their parameters.
        
        Returns:
            List of parsed obstacle dictionaries
        """
        if not self.obstacles:
            self.load_config()
        
        parsed_obstacles = []
        
        for obstacle_config in self.obstacles:
            obstacle_params = self.parse_obstacle(obstacle_config)
            if obstacle_params is not None:
                parsed_obstacles.append(obstacle_params)
        
        return parsed_obstacles
    
    def create_motion_controller(
        self,
        motion_config: Dict[str, Any],
        obstacle_position: List[float]
    ) -> Optional[MotionController]:
        """
        Create a motion controller based on the motion configuration.
        
        Args:
            motion_config: Motion configuration from YAML
            obstacle_position: Initial obstacle position [x, y, z]
        
        Returns:
            MotionController instance or None
        """
        if motion_config is None:
            return None
        
        motion_type = motion_config.get('type', 'linear')
        velocity = motion_config.get('velocity', 1.0)
        z_height = obstacle_position[2]
        
        if motion_type == 'circle':
            center = motion_config.get('center', obstacle_position[:2])
            radius = motion_config.get('radius', 5.0)
            angular_vel = velocity / radius if radius > 0 else 0.3
            return CircleMotionController(
                center=center,
                radius=radius,
                angular_velocity=angular_vel,
                z_height=z_height
            )
        
        elif motion_type == 'linear':
            path = motion_config.get('path', [])
            if len(path) >= 2:
                return LinearMotionController(
                    start_point=path[0],
                    end_point=path[-1],
                    velocity=velocity,
                    z_height=z_height
                )
        
        elif motion_type == 'elliptical':
            center = obstacle_position[:2]
            semi_major = motion_config.get('semi_major', 2.0)
            semi_minor = motion_config.get('semi_minor', 1.0)
            angle = motion_config.get('angle', 0.0)
            angular_vel = velocity / max(semi_major, semi_minor)
            return EllipticalMotionController(
                center=center,
                semi_major=semi_major,
                semi_minor=semi_minor,
                angle=angle,
                angular_velocity=angular_vel,
                z_height=z_height
            )
        
        elif motion_type == 'polygon':
            path = motion_config.get('path', [])
            if len(path) >= 2:
                return PolygonMotionController(
                    path_points=path,
                    velocity=velocity,
                    z_height=z_height
                )
        
        elif motion_type == 'neural_network':
            model_path = motion_config.get('model_path', None)
            return NeuralNetworkMotionController(
                model_path=model_path,
                z_height=z_height
            )
        
        return None
    
    def generate_spawn_script(self, obstacle_params: Dict[str, Any]) -> str:
        """
        Generate spawn script for a single obstacle.
        
        Args:
            obstacle_params: Parsed obstacle parameters
        
        Returns:
            Python script code string
        """
        if obstacle_params['type'] == 'person':
            return generate_person_spawn_script(
                name=obstacle_params['name'],
                character_type=obstacle_params['character_type'],
                x_pose=obstacle_params['x_pose'],
                y_pose=obstacle_params['y_pose'],
                z_pose=obstacle_params['z_pose']
            )
        else:
            return generate_usd_python_script(
                name=obstacle_params['name'],
                obstacle_type=obstacle_params['type'],
                x_pose=obstacle_params['x_pose'],
                y_pose=obstacle_params['y_pose'],
                z_pose=obstacle_params['z_pose'],
                size=obstacle_params['size'],
                color=obstacle_params['color'],
                has_motion=obstacle_params['has_motion']
            )
    
    def generate_all_spawn_scripts(self) -> str:
        """
        Generate combined spawn script for all obstacles.
        
        Returns:
            Combined Python script code string
        """
        scripts = []
        
        for obstacle in self.get_obstacle_list():
            script = self.generate_spawn_script(obstacle)
            scripts.append(script)
        
        header = '''#!/usr/bin/env python3
"""
Auto-generated Isaac Sim obstacle spawn script.
Generated by dynamic_obstacle_isaacsim_spawning package.
"""

'''
        return header + '\n\n'.join(scripts)
    
    def generate_trajectory_yaml(self, output_dir: str):
        """
        Generate trajectory YAML files for dynamic obstacles.
        Similar to Gazebo version for ROS2 control compatibility.
        
        Args:
            output_dir: Directory to save the generated YAML files
        """
        if not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)
        
        for obstacle in self.get_obstacle_list():
            if not obstacle['has_motion']:
                continue
            
            motion = obstacle['motion']
            if not motion or 'path' not in motion:
                continue
            
            path = motion['path']
            velocity = motion.get('velocity', 1.0)
            std = motion.get('std', 0.0)
            
            if len(path) < 2:
                continue
            
            points = []
            current_time = 0.0
            dt = 0.5
            
            for i in range(len(path) - 1):
                p1 = path[i]
                p2 = path[i + 1]
                
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                dist = math.sqrt(dx * dx + dy * dy)
                
                if dist < 1e-6:
                    continue
                
                duration = dist / velocity
                num_steps = int(duration / dt)
                
                if num_steps == 0:
                    num_steps = 1
                
                for step in range(num_steps):
                    alpha = step / num_steps
                    x = p1[0] + alpha * dx
                    y = p1[1] + alpha * dy
                    
                    x_noisy = x + random.gauss(0, std)
                    y_noisy = y + random.gauss(0, std)
                    z = obstacle['z_pose']
                    
                    points.append({
                        'time_from_start': float(current_time),
                        'positions': [x_noisy, y_noisy, z]
                    })
                    
                    current_time += dt
            
            # Add final point
            last_p = path[-1]
            x_noisy = last_p[0] + random.gauss(0, std)
            y_noisy = last_p[1] + random.gauss(0, std)
            z = obstacle['z_pose']
            points.append({
                'time_from_start': float(current_time),
                'positions': [x_noisy, y_noisy, z]
            })
            
            trajectory_data = {
                'obstacle_name': obstacle['name'],
                'joints': ['joint_x', 'joint_y', 'joint_z'],
                'points': points
            }
            
            filename = os.path.join(output_dir, f"{obstacle['name']}_trajectory.yaml")
            with open(filename, 'w') as f:
                yaml.dump(trajectory_data, f, default_flow_style=None)
    
    def save_spawn_script_to_file(self, output_path: str):
        """
        Generate and save spawn script to a file.
        
        Args:
            output_path: Path to save the Python script file
        """
        script_content = self.generate_all_spawn_scripts()
        
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        with open(output_path, 'w') as f:
            f.write(script_content)
