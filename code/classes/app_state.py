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

    def set_simulation(self, sim_type, version):
        if sim_type == "gazebo" and version in ("fortress", "harmonic"):
            self.world_manager = WorldManager(sim_type, version)
            self.simulationSet.emit(sim_type, version)
            self.status(f"Simulation: Gazebo {version.title()}", "success")
        else:
            self.world_manager = None

    def status(self, msg, kind="info"):
        self.statusMessage.emit(msg, kind)

    def cleanup(self):
        if self.world_manager:
            self.world_manager.cleanup()
