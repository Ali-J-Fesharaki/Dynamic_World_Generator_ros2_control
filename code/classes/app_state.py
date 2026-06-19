"""
Application shared state — replaces the QWizard as the central data hub.
Holds world_manager, scene items, and provides signals for cross-panel communication.
"""
from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import QGraphicsScene
from classes.world_manager import WorldManager


class AppState(QObject):
    """Shared state object passed to every panel."""

    # Signals
    worldLoaded      = pyqtSignal(str)           # world_name
    simulationSet    = pyqtSignal(str, str)       # sim_type, version
    canvasRefreshed  = pyqtSignal()
    statusMessage    = pyqtSignal(str, str)       # message, type (info/success/warning/error)
    coordinateUpdate = pyqtSignal(float, float)   # x_m, y_m

    def __init__(self):
        super().__init__()
        self.world_manager = None
        self.scene = QGraphicsScene()
        self.wall_items = {}
        self.obstacle_items = {}
        self.path_items = {}
        self.preview_process = None

    def set_simulation(self, sim_type, version):
        if sim_type == "gazebo" and version in ("fortress", "harmonic"):
            self.world_manager = WorldManager(sim_type, version)
            self.simulationSet.emit(sim_type, version)
            self.status(f"Simulation: Gazebo {version.title()}", "success")
        elif sim_type == "isaacsim":
            self.world_manager = WorldManager(sim_type, version)
            self.simulationSet.emit(sim_type, version)
            self.status(f"Simulation: Isaac Sim {version}", "success")
        else:
            self.world_manager = None

    def status(self, msg, kind="info"):
        self.statusMessage.emit(msg, kind)

    def launch_preview(self, use_ros_launch=False):
        if not self.world_manager or not self.world_manager.world_name:
            self.status("No world loaded.", "error")
            return False
            
        import threading
        
        def launch_task():
            import subprocess
            import os
            import psutil
            from utils.config import PROJECT_ROOT
            
            # Check if Isaac Sim is selected
            if getattr(self.world_manager, "sim_type", "gazebo") == "isaacsim":
                try:
                    setup = os.path.join(PROJECT_ROOT, "code", "control_ws", "install", "setup.zsh")
                    if not os.path.exists(setup):
                        setup = os.path.join(PROJECT_ROOT, "code", "control_ws", "install", "setup.bash")
                    shell = "zsh" if "zsh" in setup else "bash"
                    
                    subprocess.run(["pkill", "-9", "-f", "ros2 launch"], stderr=subprocess.DEVNULL)
                    subprocess.run(["pkill", "-9", "-f", "people.py"], stderr=subprocess.DEVNULL)
                    
                    cmd = f"source {setup} && exec ros2 launch dynamic_obstacle_isaacsim_spawning multi_obstacle_world.launch.py world_name:={self.world_manager.world_name}"
                    self.preview_process = subprocess.Popen([shell, "-c", cmd])
                    self.status("Launched Isaac Sim via ROS2", "success")
                except Exception as e:
                    self.status(f"Launch failed: {str(e)}", "error")
                return

            # Check if Gazebo is running
            gz_running = False
            for proc in psutil.process_iter(['name', 'cmdline']):
                try:
                    name = proc.info.get('name', '') or ''
                    cmdline = proc.info.get('cmdline', []) or []
                    if 'ruby' in name or 'gz' in name or 'ign' in name:
                        if any('gz' in arg or 'ign' in arg for arg in cmdline):
                            gz_running = True
                            break
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    pass
    
            if gz_running:
                # Sync models. This handles creating static models and removing ALL old models (including dynamic).
                self.world_manager.sync_static_models_to_running_gazebo()
                
                if not use_ros_launch:
                    self.status("Synced static models to Gazebo", "success")
                    return True
                else:
                    # We need to restart ROS2 nodes for dynamic obstacles, but we KEEP Gazebo running!
                    subprocess.run(["pkill", "-9", "-f", "ros2 launch"], stderr=subprocess.DEVNULL)
                    subprocess.run(["pkill", "-9", "-f", "trajectory_publisher"], stderr=subprocess.DEVNULL)
                    subprocess.run(["pkill", "-9", "-f", "robot_state_publisher"], stderr=subprocess.DEVNULL)
                    subprocess.run(["pkill", "-9", "-f", "parameter_bridge"], stderr=subprocess.DEVNULL)
            else:
                if not use_ros_launch:
                    # Starting just Gazebo for static preview
                    pass
                # Forcefully kill any lingering Gazebo/ROS processes to ensure clean start
                subprocess.run(["pkill", "-9", "-f", "ign gazebo"], stderr=subprocess.DEVNULL)
                subprocess.run(["pkill", "-9", "-f", "gz sim"], stderr=subprocess.DEVNULL)
                subprocess.run(["pkill", "-9", "ruby"], stderr=subprocess.DEVNULL)
                subprocess.run(["pkill", "-9", "-f", "ros2 launch"], stderr=subprocess.DEVNULL)
                subprocess.run(["pkill", "-9", "-f", "trajectory_publisher"], stderr=subprocess.DEVNULL)
                subprocess.run(["pkill", "-9", "-f", "robot_state_publisher"], stderr=subprocess.DEVNULL)
                subprocess.run(["pkill", "-9", "-f", "parameter_bridge"], stderr=subprocess.DEVNULL)
    
            try:
                setup = os.path.join(PROJECT_ROOT, "code", "control_ws", "install", "setup.zsh")
                if not os.path.exists(setup):
                    setup = os.path.join(PROJECT_ROOT, "code", "control_ws", "install", "setup.bash")
                shell = "zsh" if "zsh" in setup else "bash"
                
                if gz_running and use_ros_launch:
                    # Gazebo is already running, just spawn the dynamic obstacles using ros2 launch
                    cmd = f"source {setup} && exec ros2 launch dynamic_obstacle_gz_spawning multi_obstacle_world.launch.py world_name:={self.world_manager.world_name}"
                    self.preview_process = subprocess.Popen([shell, "-c", cmd])
                    self.status("Launched ROS2 spawner", "success")
                elif not gz_running:
                    # Gazebo is NOT running, ALWAYS launch via ros2 launch to ensure all plugins load!
                    cmd = f"source {setup} && exec ros2 launch dynamic_obstacle_gz_spawning multi_obstacle_world.launch.py world_name:={self.world_manager.world_name}"
                    self.preview_process = subprocess.Popen([shell, "-c", cmd])
                    self.status("Launched Gazebo via ROS2", "success")
                    
            except Exception as e:
                self.status(f"Launch failed: {str(e)}", "error")

        self.status("Initializing Launch Process...", "info")
        threading.Thread(target=launch_task, daemon=True).start()
        return True

    def cleanup(self):
        import subprocess
        if self.world_manager:
            self.world_manager.cleanup()
        if self.preview_process:
            try:
                self.preview_process.kill()
            except:
                pass
            self.preview_process = None
            
        subprocess.run(["pkill", "-9", "-f", "ign gazebo"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "gz sim"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "ruby"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "ros2 launch"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "trajectory_publisher"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "robot_state_publisher"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "parameter_bridge"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "people.py"], stderr=subprocess.DEVNULL)
