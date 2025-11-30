# Dynamic Obstacle Isaac Sim Spawning

This ROS2 package provides dynamic obstacle spawning and motion control for NVIDIA Isaac Sim.

## Overview

This package is the Isaac Sim equivalent of `dynamic_obstacle_gz_spawning` for Gazebo. It enables:

- Spawning obstacles (box, cylinder, sphere) in Isaac Sim
- Spawning animated person characters using Pegasus Simulator
- Multiple motion patterns (circle, linear, elliptical, polygon)
- Neural network motion controller interface for ML-based motion
- ROS2 integration via Isaac Sim's ROS2 Bridge

## Package Structure

```
dynamic_obstacle_isaacsim_spawning/
├── config/
│   └── obstacles.yaml          # Obstacle configuration
├── launch/
│   └── isaacsim_obstacle_spawner.launch.py
├── isaacsim_scripts/
│   ├── utils.py                # Core utilities and motion controllers
│   ├── isaacsim_spawner.py     # ROS2 spawner node
│   ├── isaacsim_standalone.py  # Standalone Isaac Sim script
│   └── trajectory_publisher.py # Trajectory publishing node
└── package.xml
```

## Prerequisites

### For ROS2 Node Usage
- ROS2 (Humble/Jazzy)
- Isaac Sim 4.5+ with ROS2 Bridge enabled
- Python 3.10+

### For Standalone Usage
- Isaac Sim 4.5+ (4.5.0 or later)
- Run from Isaac Sim's Python environment

### For Person Characters
- [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator) extension

## Installation

### Build the ROS2 Package

```bash
cd code/control_ws
colcon build --packages-select dynamic_obstacle_isaacsim_spawning
source install/setup.bash
```

## Usage

### 1. ROS2 Launch (Recommended)

Start Isaac Sim with ROS2 Bridge enabled, then:

```bash
ros2 launch dynamic_obstacle_isaacsim_spawning isaacsim_obstacle_spawner.launch.py
```

With custom configuration:

```bash
ros2 launch dynamic_obstacle_isaacsim_spawning isaacsim_obstacle_spawner.launch.py \
    config_file:=/path/to/your/obstacles.yaml
```

### 2. Standalone Isaac Sim Script

Run directly in Isaac Sim without ROS2:

```bash
# Using Isaac Sim's Python
~/.local/share/ov/pkg/isaac_sim-*/python.sh \
    code/control_ws/src/dynamic_obstacle_isaacsim_spawning/isaacsim_scripts/isaacsim_standalone.py \
    --config config/obstacles.yaml
```

### 3. Generate Spawn Script Only

Generate a Python script that can be executed inside Isaac Sim:

```bash
ros2 service call /generate_spawn_script std_srvs/srv/Trigger
```

## Configuration

### Obstacle Types

| Type | Size Parameters | Description |
|------|-----------------|-------------|
| `box` | `[width, length, height]` | Rectangular cuboid |
| `cylinder` | `[radius, height]` | Vertical cylinder |
| `sphere` | `[radius]` | Sphere |
| `person` | N/A | Animated character (requires Pegasus) |

### Motion Types

| Type | Parameters | Description |
|------|------------|-------------|
| `circle` | `center`, `radius`, `velocity` | Circular motion |
| `linear` | `path`, `velocity` | Back-and-forth linear motion |
| `elliptical` | `semi_major`, `semi_minor`, `angle`, `velocity` | Elliptical path |
| `polygon` | `path`, `velocity` | Multi-waypoint polygon path |
| `neural_network` | `model_path` | ML-based motion (experimental) |

### Example Configuration

```yaml
obstacles:
  # Dynamic box with circular motion
  - name: moving_box
    type: box
    color: red
    enabled: true
    x_pose: 3.0
    y_pose: 0.0
    z_pose: 0.5
    size: [1.0, 1.0, 1.0]
    motion:
      type: circle
      center: [0.0, 0.0]
      radius: 3.0
      velocity: 1.0

  # Walking person (requires Pegasus Simulator)
  - name: pedestrian_1
    type: person
    character_type: male_adult_construction
    enabled: true
    x_pose: 5.0
    y_pose: 0.0
    z_pose: 0.0
    size: []
    motion:
      type: linear
      velocity: 1.5
      path:
        - [5.0, 0.0]
        - [5.0, 10.0]
```

## ROS2 Control Integration

### Isaac Sim ROS2 Bridge

Isaac Sim provides native ROS2 support through its ROS2 Bridge extension. This package uses:

1. **Pose-based control**: Publishes `geometry_msgs/PoseStamped` messages
2. **Motion controllers**: Built-in Python motion controllers that update obstacle transforms

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/isaacsim/spawn_command` | `std_msgs/String` | JSON spawn commands |
| `/obstacle/{name}/target_pose` | `geometry_msgs/PoseStamped` | Pose updates for dynamic obstacles |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/spawn_all_obstacles` | `std_srvs/Trigger` | Spawn all configured obstacles |
| `/generate_spawn_script` | `std_srvs/Trigger` | Generate Python spawn script |

## Person Characters

The package supports spawning animated person characters using NVIDIA's character assets. When Pegasus Simulator is available, the following character types can be used:

- `male_adult_construction`
- `female_adult_business`
- `male_adult_business`
- `female_adult_casual`
- `male_adult_casual`

These characters can follow the same motion patterns as geometric obstacles.

## Neural Network Motion Controller

The package includes an interface for neural network-based motion control. This allows obstacles to move based on ML model predictions:

```yaml
motion:
  type: neural_network
  model_path: "/path/to/model.onnx"
```

The neural network receives Isaac Sim environment variables:
- Current position
- Nearby obstacle positions
- Target position (if available)
- Current velocity

**Note**: This is an experimental feature. Model loading and inference implementation is a placeholder for future development.

## Comparison with Gazebo Package

| Feature | Gazebo Package | Isaac Sim Package |
|---------|----------------|-------------------|
| Model Format | XACRO/SDF | USD/Python API |
| Motion Control | ROS2 Controllers | Native Python + Poses |
| Person Support | No | Yes (via Pegasus) |
| Physics | Gazebo Physics | PhysX |
| Visualization | Gazebo GUI | Isaac Sim GUI |

## Troubleshooting

### Isaac Sim Not Found

Ensure Isaac Sim is properly installed and the Python environment is configured:

```bash
# Check Isaac Sim installation
ls ~/.local/share/ov/pkg/isaac_sim-*
```

### Pegasus Simulator Not Available

Person spawning requires the Pegasus Simulator extension. Without it, geometric obstacles will still work.

### ROS2 Bridge Issues

Ensure the ROS2 Bridge extension is enabled in Isaac Sim:

1. Window > Extensions
2. Search for "ROS2 Bridge"
3. Enable the extension

## License

MIT License - See LICENSE file for details.

## Authors

- Ali Jafari Fesharaki (ali.jafari.fesh@gmail.com)

## Acknowledgments

- Based on Pegasus Simulator's people.py example by Marcelo Jacinto
- NVIDIA Isaac Sim team for the simulation platform
