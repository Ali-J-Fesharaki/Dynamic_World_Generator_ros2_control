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
        else:
            self.world_manager = None

    def status(self, msg, kind="info"):
        self.statusMessage.emit(msg, kind)

    def launch_preview(self, use_ros_launch=False):
        if not self.world_manager or not self.world_manager.world_name:
            self.status("No world loaded.", "error")
            return False
            
        import subprocess
        import os
        from utils.config import PROJECT_ROOT
        
        # Kill existing process if running
        if self.preview_process is not None:
            try:
                self.preview_process.terminate()
                self.preview_process.wait(timeout=2)
            except:
                try:
                    self.preview_process.kill()
                except:
                    pass
            self.preview_process = None

        # Forcefully kill any lingering Gazebo processes to ensure world reloads
        subprocess.run(["pkill", "-9", "-f", "ign gazebo|gz sim"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "ruby"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "ros2 launch"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "trajectory_publisher"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-9", "-f", "robot_state_publisher"], stderr=subprocess.DEVNULL)

        try:
            if use_ros_launch:
                setup = os.path.join(PROJECT_ROOT, "code", "control_ws", "install", "setup.zsh")
                if not os.path.exists(setup):
                    setup = os.path.join(PROJECT_ROOT, "code", "control_ws", "install", "setup.bash")
                shell = "zsh" if "zsh" in setup else "bash"
                cmd = f"source {setup} && exec ros2 launch dynamic_obstacle_gz_spawning multi_obstacle_world.launch.py world_name:={self.world_manager.world_name}"
                self.preview_process = subprocess.Popen([shell, "-c", cmd])
                self.status("Launched ROS2 spawner", "success")
            else:
                sdf_path = os.path.join(PROJECT_ROOT, "code", "control_ws", "install",
                                        "dynamic_obstacle_gz_spawning", "share",
                                        "dynamic_obstacle_gz_spawning", "worlds", f"{self.world_manager.world_name}.sdf")
                
                if self.world_manager.version == "harmonic":
                    cmd = ["gz", "sim", sdf_path]
                else:
                    cmd = ["ign", "gazebo", sdf_path]
                    
                self.preview_process = subprocess.Popen(cmd)
                self.status("Launched Gazebo Preview", "success")
                
            return True
        except Exception as e:
            self.status(f"Launch failed: {str(e)}", "error")
            return False

    def cleanup(self):
        if self.world_manager:
            self.world_manager.cleanup()
        if self.preview_process:
            try:
                self.preview_process.kill()
            except:
                pass
            self.preview_process = None
