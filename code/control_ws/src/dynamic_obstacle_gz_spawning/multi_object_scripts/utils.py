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
import yaml
import tempfile
import math
import random
from typing import Literal, Tuple, Optional, Dict, Any, List

"""
Loads a base Gazebo-ROS bridge YAML file and rewrites the topic names to include a robot-specific namespace.
Also updates message types from TwistStamped to Twist, if needed.
This utility adjusts TurtleBot3’s default bridge YAML config to support namespaces for ROS2 and gz.
The provided YAML file is reused but modified at runtime to prefix each topic with a namespace.
This is essential in multi-robot simulation, where each robot must publish and subscribe to its
own isolated set of topics (e.g., /tb0_1/cmd_vel) to avoid collisions.
It also normalizes message types (e.g., TwistStamped → Twist) where needed for compatibility.


Args:
    base_yaml_path (str): Path to the base YAML bridge configuration.
    namespace (str): Namespace prefix to apply to all ROS and Gazebo topic names.

Returns:
    str: Path to the modified, namespaced YAML file saved in /tmp.
"""

def create_namespaced_bridge_yaml(base_yaml_path, namespace):
    """Create a temporary namespaced bridge YAML for ros_gz_bridge."""
    with open(base_yaml_path, 'r') as f:
        bridges = yaml.safe_load(f)

    if namespace and not namespace.endswith('/'):
        namespace_with_slash = namespace + '/'
    else:
        namespace_with_slash = namespace

    namespaced_bridges = []
    for bridge in bridges:
        if bridge['ros_topic_name'] not in ['clock']:
            bridge['ros_topic_name'] = f"{namespace_with_slash}{bridge['ros_topic_name']}"
        if bridge['gz_topic_name'] not in ['clock']:
            bridge['gz_topic_name'] = f"{namespace_with_slash}{bridge['gz_topic_name']}"
        namespaced_bridges.append(bridge)

    output_path = f"/tmp/{namespace.strip('/')}_bridge.yaml"
    with open(output_path, 'w') as f:
        yaml.dump(namespaced_bridges, f)

    return output_path

"""
This function loads an SDF model file and updates its topic definitions by prefixing them with a namespace.
Many simulation models (like TurtleBot3) have hardcoded topic names (e.g., <topic>cmd_vel</topic>).
To run multiple robots simultaneously, these topics must be isolated per robot using namespaces.  The gz plugins suppose
to prefix them with robot name but it's not. This function ensures that all relevant topic tags (for turtlebot3) in the SDF (like cmd_vel, odom, imu, etc.)
are updated accordingly so each instance operates independently in the simulation. 
For custom models, user must update below text accordingly.

Args:
    model_path (str): Path to the original SDF model file.
    namespace (str): Namespace to prepend to each topic.

Returns:
    str: Modified SDF content with namespaced topics.
"""
def load_sdf_with_namespace(model_path, namespace):
    """Patch SDF file to inject robot namespace into all relevant topic tags."""
    with open(model_path, 'r') as f:
        sdf_text = f.read()

    topic_map = {
        '<tf_topic>/tf</tf_topic>': f'<tf_topic>{namespace}/tf</tf_topic>',
        '<topic>cmd_vel</topic>': f'<topic>{namespace}/cmd_vel</topic>',
        '<odom_topic>odom</odom_topic>': f'<odom_topic>{namespace}/odom</odom_topic>',
        '<topic>joint_states</topic>': f'<topic>{namespace}/joint_states</topic>',
        '<topic>imu</topic>': f'<topic>{namespace}/imu</topic>',
        '<topic>scan</topic>': f'<topic>{namespace}/scan</topic>',
        '<topic>camera/image_raw</topic>': f'<topic>{namespace}/camera/image_raw</topic>',
        '<camera_info_topic>camera/camera_info</camera_info_topic>': f'<camera_info_topic>{namespace}/camera/camera_info</camera_info_topic>',
    }

    for original, replacement in topic_map.items():
        sdf_text = sdf_text.replace(original, replacement)

    return sdf_text

"""
This function generates a namespaced RViz configuration file for a TurtleBot3 robot.
"""
def generate_rviz_config(robot_name, base_config_path):
    # Read the base RViz config
    with open(base_config_path, 'r') as f:
        config = f.read()

    # Replace placeholders
    config = config.replace('<robot_name>', f'/{robot_name}')

    # Use system temp directory
    temp_dir = tempfile.gettempdir()
    output_config_path = os.path.join(temp_dir, f'{robot_name}_rviz_config.rviz')

    with open(output_config_path, 'w') as f:
        f.write(config)

    return output_config_path

def generate_nav_config(robot_name, base_config_path):
    # Read the base nav config
    with open(base_config_path, 'r') as f:
        config = f.read()

    # Replace placeholders
    config = config.replace('<robot_name>', f'{robot_name}')

    # Use system temp directory
    temp_dir = tempfile.gettempdir()
    output_config_path = os.path.join(temp_dir, f'{robot_name}_nav_config.yaml')

    with open(output_config_path, 'w') as f:
        f.write(config)

    return output_config_path

class ObstacleLoader:
    """
    Loads obstacle configurations from YAML and generates SDF models.
    """
    
    def __init__(
        self,
        config_path: str,
        sim_version: Literal["fortress", "harmonic"] = "fortress",
        controllers_config_path: Optional[str] = None
    ):
        """
        Initialize the ObstacleLoader.
        
        Args:
            config_path: Path to obstacles.yaml configuration file
            sim_version: Gazebo simulator version ('fortress' or 'harmonic')
            controllers_config_path: Path to ros2_controllers.yaml (for dynamic obstacles)
        """
        self.config_path = config_path
        self.sim_version = sim_version
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
    
    def parse_obstacle(self, obstacle_config: Dict[str, Any]) -> Dict[str, Any]:
        """
        Parse obstacle configuration into standardized format.
        
        Args:
            obstacle_config: Raw obstacle configuration from YAML
        
        Returns:
            Parsed obstacle parameters
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
            # size: [width, length, height]
            size = tuple(size_list[:3])
        elif obstacle_type == 'cylinder':
            # size: [radius, length]
            size = tuple(size_list[:2])
        elif obstacle_type == 'sphere':
            # size: [radius]
            size = tuple(size_list[:1])
        else:
            raise ValueError(f"Unknown obstacle type: {obstacle_type}")
        
        # Extract color (optional)
        color = obstacle_config.get('color', 'gray')
        
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
            'has_motion': has_motion,
            'motion': motion_config
        }
    
    def generate_obstacle_xacro(self, obstacle_params: Dict[str, Any]) -> str:
        """
        Generate XACRO XML for a single obstacle.
        
        Args:
            obstacle_params: Parsed obstacle parameters
        
        Returns:
            XACRO XML string
        """
        return generate_obstacle_xacro(
            name=obstacle_params['name'],
            obstacle_type=obstacle_params['type'],
            x_pose=obstacle_params['x_pose'],
            y_pose=obstacle_params['y_pose'],
            z_pose=obstacle_params['z_pose'],
            size=obstacle_params['size'],
            color=obstacle_params['color'],
            has_motion=obstacle_params['has_motion'],
            sim_version=self.sim_version,
            controllers_config_path=self.controllers_config_path
        )
    
    def generate_all_obstacles_xacro(self) -> str:
        """
        Generate XACRO XML for all obstacles in the configuration.
        
        Returns:
            Combined XACRO XML string for all enabled obstacles
        """
        if not self.obstacles:
            self.load_config()
        
        xacro_models = []
        
        for obstacle_config in self.obstacles:
            obstacle_params = self.parse_obstacle(obstacle_config)
            
            if obstacle_params is None:
                # Skip disabled obstacles
                continue
            
            xacro = self.generate_obstacle_xacro(obstacle_params)
            xacro_models.append(xacro)
        
        return '\n\n'.join(xacro_models)
    
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
    
    def generate_trajectory_yaml(self, output_dir: str):
        """
        Generate trajectory YAML files for dynamic obstacles.
        
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
            
            # Skip if path has less than 2 points
            if len(path) < 2:
                continue

            points = []
            current_time = 0.0
            dt = 0.5  # Time step for interpolation
            
            # Initial point
            start_x, start_y = path[0]
            # We don't add noise to the very first point to ensure it starts where it spawns?
            # Or maybe we do. Let's add noise.
            
            # Iterate through path segments
            for i in range(len(path) - 1):
                p1 = path[i]
                p2 = path[i+1]
                
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                dist = math.sqrt(dx*dx + dy*dy)
                
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
                    
                    # Add noise
                    x_noisy = x + random.gauss(0, std)
                    y_noisy = y + random.gauss(0, std)
                    z = obstacle['z_pose'] # Keep Z constant or from config? 
                    # The obstacle has z_pose. The path is 2D [x, y].
                    
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

            # Structure for YAML
            # This structure mimics a generic trajectory definition
            trajectory_data = {
                'obstacle_name': obstacle['name'],
                'joints': ['joint_x', 'joint_y', 'joint_z'],
                'points': points
            }
            
            filename = os.path.join(output_dir, f"{obstacle['name']}_trajectory.yaml")
            with open(filename, 'w') as f:
                yaml.dump(trajectory_data, f, default_flow_style=None)

    def save_xacro_to_file(self, output_path: str):
        """
        Generate and save XACRO to a file.
        
        Args:
            output_path: Path to save the XACRO file
        """
        xacro_content = self.generate_all_obstacles_xacro()
        
        # Create directory if it doesn't exist
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        with open(output_path, 'w') as f:
            f.write(xacro_content)


def get_color(color_name: str) -> Tuple[float, float, float]:
    """
    Convert color name to RGB values.
    
    Args:
        color_name: Name of the color (e.g., 'red', 'blue', 'green')
    
    Returns:
        Tuple of RGB values (0-1 range)
    """
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
    return colors.get(color_name.lower(), (0.5, 0.5, 0.5))


def calculate_inertia(
    obstacle_type: str,
    size: Tuple[float, ...],
    density: float = 1000.0
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


def generate_geometry_xml(obstacle_type: str, size: Tuple[float, ...]) -> str:
    """
    Generate geometry XML for different obstacle types.
    
    Args:
        obstacle_type: Type of obstacle ('box', 'cylinder', 'sphere')
        size: Size parameters
    
    Returns:
        XML string for geometry
    """
    if obstacle_type == "box":
        w, l, h = size
        return f"<box size=\"{w:.6f} {l:.6f} {h:.6f}\"/>"
    elif obstacle_type == "cylinder":
        r, h = size
        return f"<cylinder radius=\"{r:.6f}\" length=\"{h:.6f}\"/>"
    elif obstacle_type == "sphere":
        r = size[0]
        return f"<sphere radius=\"{r:.6f}\"/>"
    else:
        return "<box size=\"1 1 1\"/>"


def generate_ros2_control_plugin(
    sim_version: Literal["fortress", "harmonic"],
    controllers_config_path: str,
    obstacle_name: str
) -> str:
    """
    Generate the ros2_control plugin block based on simulator version.
    
    Args:
        sim_version: Either "fortress" or "harmonic"
        controllers_config_path: Path to the ros2_controllers.yaml file
        obstacle_name: Name of the obstacle for namespacing
    
    Returns:
        XML string containing the plugin configuration
    """
    joint_x = f"joint_x"
    joint_y = f"joint_y"
    joint_z = f"joint_z"
    
    if sim_version == "fortress":
        param_tag = f"<parameters>{controllers_config_path}</parameters>"
        control_name = f"{obstacle_name}_controller"
    else:  # harmonic
        param_tag = f"<parameters>{controllers_config_path}</parameters>"
        control_name = f"{obstacle_name}_control"
    
    return f'''      <ros2_control name="{obstacle_name}" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
      </hardware>
      <joint name="{joint_x}">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
      <joint name="{joint_y}">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
      <joint name="{joint_z}">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
    </ros2_control>
    <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <ros>
        <namespace>$(arg namespace)</namespace>
        <remapping>/tf:=tf</remapping>
        <remapping>/tf_static:=tf_static</remapping>
      </ros>
      {param_tag}
    </plugin>
    </gazebo>'''


def generate_prismatic_joints(obstacle_name: str, base_z: float = 0.0) -> str:
    """
    Generate three orthogonal unlimited-range prismatic joints for XYZ motion.
    
    Args:
        obstacle_name: Name of the obstacle model
        base_z: Z-coordinate for the base link (default 0.0)
    
    Returns:
        XML string containing link and joint definitions
    """
    return f'''
    <!-- Prismatic joint structure for {obstacle_name} -->
    <link name="$(arg namespace)/link_x">
      <inertial>
        <origin xyz="0 0 {base_z:.6f}" rpy="0 0 0"/>
        <mass value="0.001"/>
        <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
      </inertial>
    </link>
    
    <link name="$(arg namespace)/link_y">
      <inertial>
        <origin xyz="0 0 {base_z:.6f}" rpy="0 0 0"/>
        <mass value="0.001"/>
        <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
      </inertial>
    </link>
    
    <link name="$(arg namespace)/link_z">
      <inertial>
        <origin xyz="0 0 {base_z:.6f}" rpy="0 0 0"/>
        <mass value="0.001"/>
        <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
      </inertial>
    </link>

    <joint name="joint_x" type="prismatic">
      <parent link="world"/>
      <child link="$(arg namespace)/link_x"/>
      <axis xyz="1 0 0"/>
      <limit lower="-1e9" upper="1e9" effort="10000.0" velocity="1000.0"/>
      <dynamics damping="0.0" friction="0.0"/>
    </joint>

    <joint name="joint_y" type="prismatic">
      <parent link="$(arg namespace)/link_x"/>
      <child link="$(arg namespace)/link_y"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1e9" upper="1e9" effort="1000.0" velocity="100.0"/>
      <dynamics damping="0.0" friction="0.0"/>
    </joint>

    <joint name="joint_z" type="prismatic">
      <parent link="$(arg namespace)/link_y"/>
      <child link="$(arg namespace)/link_z"/>
      <axis xyz="0 0 1"/>
      <limit lower="-1e9" upper="1e9" effort="1000.0" velocity="100.0"/>
      <dynamics damping="0.0" friction="0.0"/>
    </joint>

    <joint name="fixed_joint" type="fixed">
      <parent link="$(arg namespace)/link_z"/>
      <child link="$(arg namespace)/link"/>
    </joint>
'''

def generate_obstacle_xacro_macro(
    obstacle_type: str,
    sim_version: Literal["fortress", "harmonic"] = "fortress"
) -> str:
    """
    Generate a XACRO macro definition for reusable obstacle models.
    
    Args:
        obstacle_type: Type of obstacle ('box', 'cylinder', 'sphere')
        sim_version: Gazebo version ('fortress' or 'harmonic')
    
    Returns:
        XACRO macro definition string
    """
    # Note: sim_version parameter reserved for future use
    _ = sim_version  # Mark as intentionally unused
    
    if obstacle_type == "box":
        params = "name x_pose y_pose z_pose width length height color has_motion:=false controllers_config:=''"
    elif obstacle_type == "cylinder":
        params = "name x_pose y_pose z_pose radius length color has_motion:=false controllers_config:=''"
    elif obstacle_type == "sphere":
        params = "name x_pose y_pose z_pose radius color has_motion:=false controllers_config:=''"
    else:
        params = "name x_pose y_pose z_pose color has_motion:=false controllers_config:=''"
        size_expr = "1 1 1"
    
    return f'''<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <xacro:macro name="{obstacle_type}_obstacle" params="{params}">
    <!-- This macro generates a {obstacle_type} obstacle that can be static or dynamic -->
    
    <!-- Call the Python function to generate the full model -->
    <!-- Note: In practice, XACRO macros are typically expanded at parse time -->
    <!-- This is a template showing the structure -->
  </xacro:macro>

</robot>'''


def generate_obstacle_xacro(
    name: str,
    obstacle_type: str,
    x_pose: float,
    y_pose: float,
    z_pose: float,
    size: Tuple[float, ...],
    color: str = "gray",
    has_motion: bool = False,
    sim_version: Literal["fortress", "harmonic"] = "fortress",
    controllers_config_path: Optional[str] = None
) -> str:
    """
    Generate complete XACRO model for an obstacle.
    
    Args:
        name: Unique name for the obstacle
        obstacle_type: Type of obstacle ('box', 'cylinder', 'sphere')
        x_pose: X position in world frame
        y_pose: Y position in world frame
        z_pose: Z position in world frame
        size: Size parameters (depends on obstacle_type)
              - box: (width, length, height)
              - cylinder: (radius, length)
              - sphere: (radius,)
        color: Color name (default 'gray')
        has_motion: Whether obstacle is dynamic (default False)
        sim_version: Gazebo version ('fortress' or 'harmonic')
        controllers_config_path: Path to ros2_controllers.yaml (required if has_motion=True)
    
    Returns:
        Complete XACRO XML string for the obstacle model
    """
    color_rgb = get_color(color)
    
    # Start XACRO definition with robot tag
    xacro = f'''<?xml version="1.0"?>
<robot name="{name}">
    <arg namespace="namespace" default="namespace" />
    <link name="world"/>
'''
    
    # Add prismatic joints for dynamic obstacles BEFORE the main link
    if has_motion:
        xacro += generate_prismatic_joints(name, base_z=0.0)
    
    # Main link definition - use consistent naming
    xacro += f'''
    <link name="$(arg namespace)/link">'''
    
    # Geometry
    geometry_xml = generate_geometry_xml(obstacle_type, size)
    
    # Add origin for the link (initial position)
    xacro += f'''
      <origin xyz="{x_pose:.6f} {y_pose:.6f} {z_pose:.6f}" rpy="0 0 0"/>
      <collision name="collision">
        <geometry>
          {geometry_xml}
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          {geometry_xml}
        </geometry>
        <material name="{color}">
          <color rgba="{color_rgb[0]} {color_rgb[1]} {color_rgb[2]} 1"/>
        </material>
      </visual>'''
    
    # Add inertial properties (always needed in URDF, even for static obstacles)
    mass, ixx, iyy, izz = calculate_inertia(obstacle_type, size)
    if not has_motion:
        # For static obstacles, use very small mass
        mass = 0.001
        ixx = iyy = izz = 0.000001
    
    xacro += f'''
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="{mass:.6f}"/>
        <inertia ixx="{ixx:.6f}" ixy="0" ixz="0" iyy="{iyy:.6f}" iyz="0" izz="{izz:.6f}"/>
      </inertial>
    </link>
'''
    
    # For static obstacles, add a fixed joint to world
    if not has_motion:
        xacro += f'''
    <joint name="fixed_to_world" type="fixed">
      <parent link="$(arg namespace)/world"/>
      <child link="$(arg namespace)/link"/>
      <origin xyz="{x_pose:.6f} {y_pose:.6f} {z_pose:.6f}" rpy="0 0 0"/>
    </joint>
'''
    
    # Add Gazebo tags for visualization and physics
    xacro += f'''
    <gazebo reference="$(arg namespace)/link">
      <material>Gazebo/{color.capitalize()}</material>'''
    
    if not has_motion:
        xacro += '''
      <static>true</static>'''
    else:
        xacro += '''
      <gravity>false</gravity>'''
    
    xacro += '''
    </gazebo>
'''
    
    # Add ros2_control plugin for dynamic obstacles
    if has_motion and controllers_config_path:
        xacro += "\n" + generate_ros2_control_plugin(sim_version, controllers_config_path, name)
    
    xacro += '''
</robot>'''
    
    return xacro
