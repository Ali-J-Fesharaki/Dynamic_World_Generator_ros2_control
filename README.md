# Dynamic World Generator Wizard (V2)

![Dynamic World Generator Wizard Banner](https://github.com/user-attachments/assets/1b00aa22-24d7-40f1-8526-a3612bd7f503)

**Dynamic World Generator Wizard** is a *PyQt5*-based graphical user interface (*GUI*) application designed to create and manage dynamic simulation worlds for *Gazebo* (*Harmonic* or *Fortress* versions). It allows users to build custom worlds with walls, static obstacles (boxes, cylinders, spheres), and dynamic obstacles with various motion paths (linear, elliptical, polygon).

**Major Update (V2):** The backend has been completely re-engineered to use **ROS 2 Control**. Unlike the previous version which only visualized moving objects, this version ensures **superior dynamic obstacle collision handling**. Obstacles are spawned and controlled via ROS 2, providing realistic physics interactions and collision responses, making it ideal for testing autonomous navigation and collision avoidance algorithms.

The wizard guides users through a step-by-step process, ensuring an intuitive experience. It supports creating new worlds from empty templates, loading existing ones, and applying changes in real-time to *Gazebo*.

🙏 A special thanks to **Professor Sousso KELOUWANI** for his excellent idea that inspired the creation of this application.

## Installation and Usage

### Prerequisites

* **OS**: Ubuntu 22.04 (Humble) or 24.04 (Jazzy).
* **ROS 2**: Installed and configured (Humble or Jazzy).
* **Python**: *3.10+*.
* **Dependencies**: Install required libraries:
  ```bash
  pip install PyQt5 lxml psutil
  ```
* **Gazebo**: Install *Gazebo Harmonic* (recommended) or *Fortress*.
  * For *Harmonic* (*Ubuntu*/*Debian*), please visit:
    [https://gazebosim.org/docs/harmonic/install_ubuntu/](https://gazebosim.org/docs/harmonic/install_ubuntu/)

  * For *Fortress*, please visit:
    [https://gazebosim.org/docs/fortress/install_ubuntu/](https://gazebosim.org/docs/fortress/install_ubuntu/)

  * For *Harmonic*, also install transport libraries:
    ```bash
    pip install gz-transport13 gz-msgs10
    ```
* **ROS 2 Packages**: Ensure `ros2_control`, `ros2_controllers`, and `gazebo_ros_pkgs` are installed.
  ```bash
  sudo apt install ros-<distro>-ros2-control ros-<distro>-ros2-controllers ros-<distro>-gazebo-ros-pkgs
  ```
* **Images and Worlds**: Ensure the `images/intro/`, `images/future/`, and `worlds/gazebo/{version}/empty_world.sdf` directories exist in the project root.

### Setup

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/Ali-J-Fesharaki/Dynamic_World_Generator_ros2_control.git
   cd Dynamic_World_Generator_ros2_control
   ```

2. **Build the ROS 2 Workspace**:
   ```bash
   cd code/control_ws
   colcon build
   source install/setup.bash
   ```

3. **Run the Application**:
   Run the main file from the `code/` directory:
   ```bash
   cd ../
   python3 dwg_wizard.py
   ```

### Troubleshooting Installation

* **PyQt5 Errors**: Ensure a display server is running (e.g., on *WSL*, use `export DISPLAY=:0` or install an *X server* like *Xming*).
* **Gazebo Not Found**: Verify installation:
  ```bash
  gz sim --version  # For Harmonic
  ign gazebo --version  # For Fortress
  ```
* **ROS 2 Issues**: Ensure you have sourced your ROS 2 installation (`source /opt/ros/<distro>/setup.bash`) and the local workspace (`source code/control_ws/install/setup.bash`).
* **Missing SDF Files**: Ensure `empty_world.sdf` exists in `worlds/gazebo/{version}/`.
* **Path Issues**: If images or worlds are not found, verify paths in `code/utils/config.py`. Update `PROJECT_ROOT` if the project is moved.

## Tutorial: Creating a Complete Dynamic World

The wizard guides you through a step-by-step process to build a dynamic world. Below is a detailed tutorial covering all options and features.

### Step 1: Welcome Page

<img width="1857" height="1048" alt="Welcome Page" src="https://github.com/user-attachments/assets/7bbd4df6-e7d6-4bad-8743-cbef0037cfc5" />

### Step 2: Select Simulation Platform

* **Choose Simulation**:
  * **Gazebo Harmonic (Recommended)**: Select for the latest features. Recommended for the best outcome and results.
  * **Gazebo Fortress**: Supported, but Harmonic is preferred for better integration.
  * **Isaac Sim**: Under development, currently disabled.
  * Click *Next* when done.

<img width="1857" height="1048" alt="Simulation Selection" src="https://github.com/user-attachments/assets/7d7aa432-506c-49c9-9ad2-50e3b1d29ad7" />

### Step 3: Design Walls

* **Create or Load World**:
  * Enter a world name (e.g., `myWorld`) in the text field.
  * Click *Create New World* to copy `empty_world.sdf` or *Load World* to open an existing *SDF* file.
* **Add Walls**:
  * Set width (*m*, e.g., *0.2*), height (*m*, e.g., *1.5*), and color (*Black*, *Gray*, *White*, *Red*, *Blue*, *Green*).
  * Click on the canvas twice to draw a wall (start and end points).
  * Walls appear as lines on the canvas.
* **Remove Walls**: Select a wall from the list and click *Remove Selected Wall*.
* **Apply Changes**: Click *Apply and Preview* to update the *Gazebo* simulation and save to the *SDF* file (`worlds/gazebo/{version}/myWorld.sdf`).
* **Canvas Controls**: Zoom with the mouse wheel, pan with the middle mouse button.
* Click *Next* when done.

<img width="1857" height="1048" alt="Walls Design" src="https://github.com/user-attachments/assets/522a2955-2ce6-49a9-a17e-8fd4d751ad32" />

### Step 4: Add Static Obstacles

* **Select Obstacle Type**:
  * **Box**: Set width, length, height (*m*).
  * **Cylinder**: Set radius, height (*m*).
  * **Sphere**: Set radius (*m*).
* **Customize**:
  * Choose color (*Black*, *Gray*, *White*, *Red*, *Blue*, *Green*).
  * Enter dimensions (e.g., box: *1x1x1*; cylinder: radius=*0.5*, height=*1*).
* **Add Obstacles**: Click on the canvas to place the obstacle at the desired position.
* **Remove Obstacles**: Select from the list and click *Remove Selected Obstacle*.
* **Apply Changes**: Click *Apply and Preview* to update *Gazebo* and *SDF*.
* **Canvas Controls**: Zoom/pan as before.
* Click *Next* when done.

<img width="1857" height="1048" alt="Static Obstacles" src="https://github.com/user-attachments/assets/2f1c0359-c13b-42ad-8213-d121789d3b4f" />

### Step 5: Add Dynamic Obstacles

* **Select Obstacle**: Choose a static obstacle from the list (populated from Step 4).
* **Choose Motion Type**:
  * **Linear**: Define a path with *2* points (red line).
  * **Elliptical**: Define a point to act as a guider. The direction of the semi-major axis of the ellipse will be along the line connecting the defined point and the center of the obstacle (green ellipse).
  * **Polygon**: Define multiple points, close with *Finish Path* (blue lines).
* **Customize Motion**:
  * Set velocity (*m/s*, e.g., *5.0*) and *std* (randomness, e.g., *0.1*).
  * For elliptical, set semi-major (e.g., *2.0*) and semi-minor (e.g., *1.0*) axes.
* **Define Path**:
  * Click *Start Defining Path*.
  * Click on the canvas to add points:
    * Linear: *2* clicks.
    * Elliptical: *1* click (defines orientation).
    * Polygon: Multiple clicks, then *Finish Path* to close.
  * Path appears on the canvas for preview.
* **Apply Changes**: Click *Apply and Preview*. This will:
  1.  Export the obstacle configuration to `obstacles.yaml`.
  2.  Launch the ROS 2 `multi_obstacle_world.launch.py` file in a new terminal.
  3.  Spawn the obstacles in Gazebo with `ros2_control` enabled.
* **Canvas Controls**: Zoom/pan as before.
* Click *Next* when done.

<img width="1857" height="1048" alt="Dynamic Obstacles" src="https://github.com/user-attachments/assets/b287e0b4-e7d1-41d3-8f04-daedb002bf95" />

### Step 6: Coming Soon Page

* Displays teasers for future features:
  * **Gazebo Ionic**: Upcoming *Gazebo* version.
  * **Isaac Sim 4.5.0/5.0.0**: Future simulator support.
* Click *Finish* to exit the wizard.

<img width="1857" height="1048" alt="Coming Soon" src="https://github.com/user-attachments/assets/56a646ee-42cc-4d98-b168-8dd1ad0e1214" />

---

+ If you have any questions, please let me know: **ali.jafari.fesh@gmail.com**
