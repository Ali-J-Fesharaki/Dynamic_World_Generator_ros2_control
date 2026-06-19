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
# Based on Pegasus Simulator's people.py example by Marcelo Jacinto

"""
Isaac Sim Standalone Dynamic Obstacle Spawner.

This script demonstrates how to spawn and control dynamic obstacles in Isaac Sim
without ROS2 dependencies. It uses the Isaac Sim Python API directly.

Usage:
    python3 isaacsim_standalone.py --config obstacles.yaml

Requirements:
    - Isaac Sim 4.5+ installed
    - Run from Isaac Sim's Python environment
"""

import argparse
import os
import sys
import math
import yaml
from typing import List, Dict, Any, Optional


def main():
    """Main entry point for standalone Isaac Sim obstacle spawning."""
    
    parser = argparse.ArgumentParser(description='Spawn dynamic obstacles in Isaac Sim')
    parser.add_argument('--config', type=str, default='',
                       help='Path to obstacles YAML configuration file')
    parser.add_argument('--headless', action='store_true',
                       help='Run in headless mode')
    parser.add_argument('--world', type=str, default='',
                       help='Path to world USD file to load')
    
    args = parser.parse_args()
    
    # Import Isaac Sim modules (must be done after argument parsing)
    try:
        import carb
        from isaacsim import SimulationApp
    except ImportError:
        print("Error: Isaac Sim Python modules not found.")
        print("Please run this script from Isaac Sim's Python environment:")
        print("  ~/.local/share/ov/pkg/isaac_sim-*/python.sh isaacsim_standalone.py")
        sys.exit(1)
    
    # Start Isaac Sim
    simulation_app = SimulationApp({"headless": args.headless})
    
    # Import additional modules after SimulationApp is initialized
    import omni.timeline
    from omni.isaac.core.world import World
    from isaacsim.core.utils.extensions import enable_extension
    import omni.usd
    from pxr import Gf, UsdGeom, UsdPhysics
    
    # Enable ROS2 bridge if available
    try:
        enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()
    except Exception as e:
        carb.log_warn(f"ROS2 Bridge extension not available: {e}")
    
    # Create new stage
    omni.usd.get_context().new_stage()
    
    # Import numpy after Isaac Sim is initialized
    import numpy as np
    
    # Create the World
    world = World()
    
    # Load world file if specified
    if args.world and os.path.exists(args.world):
        try:
            omni.usd.get_context().open_stage(args.world)
            carb.log_info(f"Loaded world: {args.world}")
        except Exception as e:
            carb.log_error(f"Failed to load world: {e}")
    
    # Create obstacle spawner instance
    spawner = IsaacSimObstacleSpawner(world, simulation_app)
    
    # Load configuration
    if args.config and os.path.exists(args.config):
        spawner.load_config(args.config)
        spawner.spawn_all_obstacles()
    else:
        carb.log_warn("No configuration file specified. Creating demo obstacles.")
        spawner.create_demo_obstacles()
    
    # Reset world
    world.reset()
    
    # Get timeline
    timeline = omni.timeline.get_timeline_interface()
    
    # Run simulation
    carb.log_info("Starting simulation. Press Ctrl+C to stop.")
    timeline.play()
    
    try:
        while simulation_app.is_running():
            spawner.update()
            world.step(render=True)
    except KeyboardInterrupt:
        pass
    finally:
        carb.log_warn("Shutting down simulation.")
        timeline.stop()
        simulation_app.close()


class IsaacSimObstacleSpawner:
    """
    Manages spawning and updating of dynamic obstacles in Isaac Sim.
    """
    
    def __init__(self, world, simulation_app):
        """
        Initialize the obstacle spawner.
        
        Args:
            world: Isaac Sim World instance
            simulation_app: SimulationApp instance
        """
        self.world = world
        self.simulation_app = simulation_app
        self.obstacles = []
        self.motion_controllers = {}
        self.obstacle_prims = {}
        self.last_time = 0.0
    
    def load_config(self, config_path: str):
        """
        Load obstacle configuration from YAML file.
        
        Args:
            config_path: Path to YAML configuration file
        """
        import carb
        
        if not os.path.exists(config_path):
            carb.log_error(f"Config file not found: {config_path}")
            return
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        if 'obstacles' not in config:
            carb.log_error("Config file must contain 'obstacles' key")
            return
        
        self.obstacles = []
        for obs_config in config['obstacles']:
            if obs_config.get('enabled', True):
                self.obstacles.append(obs_config)
        
        carb.log_info(f"Loaded {len(self.obstacles)} obstacles from config")
    
    def spawn_all_obstacles(self):
        """Spawn all configured obstacles."""
        for obstacle in self.obstacles:
            self._spawn_obstacle(obstacle)
    
    def _spawn_obstacle(self, obstacle_config: Dict[str, Any]):
        """
        Spawn a single obstacle based on configuration.
        
        Args:
            obstacle_config: Obstacle configuration dictionary
        """
        import carb
        from omni.isaac.core.prims import GeometryPrim, XFormPrim
        from isaacsim.core.utils.prims import create_prim, get_prim_at_path
        from pxr import Gf, UsdGeom, UsdShade, Sdf
        
        name = obstacle_config['name']
        obs_type = obstacle_config['type']
        x = obstacle_config['x_pose']
        y = obstacle_config['y_pose']
        z = obstacle_config['z_pose']
        color = obstacle_config.get('color', 'gray')
        size = obstacle_config.get('size', [1.0, 1.0, 1.0])
        
        prim_path = f"/World/{name}"
        
        try:
            if obs_type == 'box':
                # Create cube
                prim = create_prim(
                    prim_path=prim_path,
                    prim_type="Cube",
                    position=Gf.Vec3d(x, y, z),
                    scale=Gf.Vec3d(size[0], size[1], size[2])
                )
            elif obs_type == 'cylinder':
                prim = create_prim(
                    prim_path=prim_path,
                    prim_type="Cylinder",
                    position=Gf.Vec3d(x, y, z),
                    attributes={"radius": size[0], "height": size[1]}
                )
            elif obs_type == 'sphere':
                prim = create_prim(
                    prim_path=prim_path,
                    prim_type="Sphere",
                    position=Gf.Vec3d(x, y, z),
                    attributes={"radius": size[0]}
                )
            elif obs_type == 'person':
                self._spawn_person(obstacle_config)
                return
            else:
                carb.log_warn(f"Unknown obstacle type: {obs_type}")
                return
            
            self.obstacle_prims[name] = prim_path
            
            # Apply color
            self._apply_color(prim_path, color)
            
            # Setup motion controller if dynamic
            if obstacle_config.get('motion'):
                self._setup_motion_controller(name, obstacle_config)
            
            carb.log_info(f"Spawned obstacle: {name} at ({x}, {y}, {z})")
            
        except Exception as e:
            carb.log_error(f"Failed to spawn obstacle {name}: {e}")
    
    def _spawn_person(self, obstacle_config: Dict[str, Any]):
        """
        Spawn a person character using Pegasus Simulator's Person class.
        
        Args:
            obstacle_config: Person configuration dictionary
        """
        import carb
        
        name = obstacle_config['name']
        x = obstacle_config['x_pose']
        y = obstacle_config['y_pose']
        z = obstacle_config.get('z_pose', 0.0)
        character_type = obstacle_config.get('character_type', 'original_male_adult_construction_05')
        
        try:
            from pegasus.simulator.logic.people.person import Person
            from pegasus.simulator.logic.people.person_controller import PersonController
            
            # Create custom controller based on motion config
            motion_config = obstacle_config.get('motion', {})
            controller = self._create_person_controller(motion_config, [x, y, z])
            
            person = Person(
                name,
                character_type,
                init_pos=[x, y, z],
                init_yaw=0.0,
                controller=controller
            )
            
            self.obstacle_prims[name] = f"/World/{name}"
            carb.log_info(f"Spawned person: {name}")
            
        except ImportError:
            carb.log_warn(
                "Pegasus Simulator not available. "
                "Person spawning requires the Pegasus extension."
            )
    
    def _create_person_controller(self, motion_config: Dict, initial_pos: List[float]):
        """Create a PersonController based on motion configuration."""
        try:
            from pegasus.simulator.logic.people.person_controller import PersonController
            import numpy as np
            
            motion_type = motion_config.get('type', 'circle')
            
            if motion_type == 'circle':
                radius = motion_config.get('radius', 5.0)
                velocity = motion_config.get('velocity', 1.0)
                
                class CircleController(PersonController):
                    def __init__(self):
                        super().__init__()
                        self._radius = radius
                        self.gamma = 0.0
                        self.gamma_dot = velocity / radius
                    
                    def update(self, dt: float):
                        self.gamma += self.gamma_dot * dt
                        self._person.update_target_position([
                            self._radius * np.cos(self.gamma),
                            self._radius * np.sin(self.gamma),
                            0.0
                        ])
                
                return CircleController()
            
            elif motion_type == 'linear':
                path = motion_config.get('path', [[0, 0], [10, 0]])
                velocity = motion_config.get('velocity', 1.0)
                
                class LinearController(PersonController):
                    def __init__(self):
                        super().__init__()
                        self._path = path
                        self._velocity = velocity
                        self._current_target = 1
                        self._direction = 1
                    
                    def update(self, dt: float):
                        target = self._path[self._current_target]
                        self._person.update_target_position([target[0], target[1], 0.0], self._velocity)
                        
                        # Simple target switching logic
                        pos = self._person.get_position()
                        dist = np.sqrt((pos[0] - target[0])**2 + (pos[1] - target[1])**2)
                        if dist < 0.5:
                            self._current_target += self._direction
                            if self._current_target >= len(self._path):
                                self._current_target = len(self._path) - 2
                                self._direction = -1
                            elif self._current_target < 0:
                                self._current_target = 1
                                self._direction = 1
                
                return LinearController()
            
            # Default: no controller
            return None
            
        except ImportError:
            return None
    
    def _apply_color(self, prim_path: str, color_name: str):
        """
        Apply color material to a prim.
        
        Args:
            prim_path: USD prim path
            color_name: Color name
        """
        from isaacsim.core.utils.prims import get_prim_at_path
        from pxr import UsdGeom
        
        colors = {
            'red': (1.0, 0.0, 0.0),
            'green': (0.0, 1.0, 0.0),
            'blue': (0.0, 0.0, 1.0),
            'yellow': (1.0, 1.0, 0.0),
            'cyan': (0.0, 1.0, 1.0),
            'magenta': (1.0, 0.0, 1.0),
            'white': (1.0, 1.0, 1.0),
            'black': (0.0, 0.0, 0.0),
            'gray': (0.5, 0.5, 0.5),
            'orange': (1.0, 0.5, 0.0),
            'purple': (0.5, 0.0, 0.5),
        }
        
        rgb = colors.get(color_name.lower(), (0.5, 0.5, 0.5))
        
        prim = get_prim_at_path(prim_path)
        if prim and prim.IsValid():
            geom = UsdGeom.Gprim(prim)
            geom.CreateDisplayColorAttr(value=[rgb])
    
    def _setup_motion_controller(self, name: str, obstacle_config: Dict[str, Any]):
        """
        Setup motion controller for a dynamic obstacle.
        
        Args:
            name: Obstacle name
            obstacle_config: Obstacle configuration
        """
        motion = obstacle_config.get('motion', {})
        motion_type = motion.get('type', 'linear')
        velocity = motion.get('velocity', 1.0)
        position = [obstacle_config['x_pose'], obstacle_config['y_pose'], obstacle_config['z_pose']]
        
        controller_data = {
            'type': motion_type,
            'config': motion,
            'velocity': velocity,
            'position': position.copy(),
            'progress': 0.0,
            'direction': 1.0,
            'gamma': 0.0
        }
        
        self.motion_controllers[name] = controller_data
    
    def update(self):
        """Update all motion controllers and obstacle positions."""
        import carb
        from isaacsim.core.utils.prims import get_prim_at_path
        from pxr import Gf, UsdGeom
        
        # Get current simulation time
        try:
            current_time = self.world.current_time
        except Exception:
            current_time = self.last_time + 1.0 / 60.0
        
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt <= 0:
            return
        
        for name, controller in self.motion_controllers.items():
            if name not in self.obstacle_prims:
                continue
            
            prim_path = self.obstacle_prims[name]
            prim = get_prim_at_path(prim_path)
            
            if not prim or not prim.IsValid():
                continue
            
            # Calculate new position based on motion type
            new_pos = self._calculate_new_position(controller, dt)
            controller['position'] = new_pos
            
            # Update prim transform
            xformable = UsdGeom.Xformable(prim)
            ops = xformable.GetOrderedXformOps()
            
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(new_pos[0], new_pos[1], new_pos[2]))
                    break
            else:
                # No translate op found, add one
                xformable.AddTranslateOp().Set(Gf.Vec3d(new_pos[0], new_pos[1], new_pos[2]))
    
    def _calculate_new_position(self, controller: Dict, dt: float) -> List[float]:
        """
        Calculate new position based on motion controller state.
        
        Args:
            controller: Controller state dictionary
            dt: Time delta
        
        Returns:
            New position [x, y, z]
        """
        motion_type = controller['type']
        config = controller['config']
        pos = controller['position'].copy()
        velocity = controller['velocity']
        
        if motion_type == 'circle':
            center = config.get('center', [0.0, 0.0])
            radius = config.get('radius', 5.0)
            controller['gamma'] += (velocity / radius) * dt
            
            pos[0] = center[0] + radius * math.cos(controller['gamma'])
            pos[1] = center[1] + radius * math.sin(controller['gamma'])
        
        elif motion_type == 'linear':
            path = config.get('path', [[0, 0], [10, 0]])
            if len(path) >= 2:
                start = path[0]
                end = path[-1]
                
                dx = end[0] - start[0]
                dy = end[1] - start[1]
                dist = math.sqrt(dx*dx + dy*dy)
                
                if dist > 1e-6:
                    controller['progress'] += (velocity * dt * controller['direction']) / dist
                    
                    if controller['progress'] >= 1.0:
                        controller['progress'] = 1.0
                        controller['direction'] = -1.0
                    elif controller['progress'] <= 0.0:
                        controller['progress'] = 0.0
                        controller['direction'] = 1.0
                    
                    pos[0] = start[0] + controller['progress'] * dx
                    pos[1] = start[1] + controller['progress'] * dy
        
        elif motion_type == 'elliptical':
            center = pos[:2]
            semi_major = config.get('semi_major', 2.0)
            semi_minor = config.get('semi_minor', 1.0)
            angle = config.get('angle', 0.0)
            
            controller['gamma'] += (velocity / max(semi_major, semi_minor)) * dt
            
            local_x = semi_major * math.cos(controller['gamma'])
            local_y = semi_minor * math.sin(controller['gamma'])
            
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            pos[0] = center[0] + local_x * cos_a - local_y * sin_a
            pos[1] = center[1] + local_x * sin_a + local_y * cos_a
        
        return pos
    
    def create_demo_obstacles(self):
        """Create demo obstacles for testing."""
        import carb
        
        demo_config = {
            'obstacles': [
                {
                    'name': 'demo_box',
                    'type': 'box',
                    'x_pose': 3.0,
                    'y_pose': 0.0,
                    'z_pose': 0.5,
                    'size': [1.0, 1.0, 1.0],
                    'color': 'red',
                    'enabled': True,
                    'motion': {
                        'type': 'circle',
                        'radius': 3.0,
                        'velocity': 1.0
                    }
                },
                {
                    'name': 'demo_cylinder',
                    'type': 'cylinder',
                    'x_pose': -3.0,
                    'y_pose': 0.0,
                    'z_pose': 0.5,
                    'size': [0.5, 1.0],
                    'color': 'blue',
                    'enabled': True,
                    'motion': {
                        'type': 'linear',
                        'path': [[-3, 0], [-3, 5]],
                        'velocity': 1.5
                    }
                },
                {
                    'name': 'demo_sphere',
                    'type': 'sphere',
                    'x_pose': 0.0,
                    'y_pose': 5.0,
                    'z_pose': 1.0,
                    'size': [0.5],
                    'color': 'green',
                    'enabled': True
                }
            ]
        }
        
        self.obstacles = demo_config['obstacles']
        carb.log_info("Created demo obstacles configuration")


if __name__ == "__main__":
    main()
