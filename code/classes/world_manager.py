import os
from utils.config import PROJECT_ROOT, WORLDS_GAZEBO_DIR

# Isaac Sim worlds directory
WORLDS_ISAACSIM_DIR = os.path.join(os.path.dirname(WORLDS_GAZEBO_DIR), "isaacsim")

class WorldManager:
    def __init__(self, simulation, version):
        # Initialize world manager with simulation and version
        self.simulation = simulation
        self.version = version
        self.world_name = None
        self.world_path = None
        self.models = []
        self.map_path = None # Path to the background map image (PGM)

    def create_new_world(self, world_name):
        # Create a new world state (no file creation)
        self.world_name = world_name
        if self.simulation == "gazebo":
            self.world_path = os.path.join(WORLDS_GAZEBO_DIR, self.version, f"{world_name}.sdf")
        elif self.simulation == "isaacsim":
            self.world_path = os.path.join(WORLDS_ISAACSIM_DIR, self.version, f"{world_name}.usd")
        self.models = []

    def load_world(self, world_name):
        # Load an existing world (read-only for context)
        self.world_name = world_name
        if self.simulation == "gazebo":
            self.world_path = os.path.join(WORLDS_GAZEBO_DIR, self.version, f"{world_name}.sdf")
        elif self.simulation == "isaacsim":
            self.world_path = os.path.join(WORLDS_ISAACSIM_DIR, self.version, f"{world_name}.usd")
        self.models = []
        # We no longer parse SDF/USD. Models are managed in memory for the session.

    def add_model(self, model):
        # Add or update a model in the world
        for existing_model in self.models:
            if existing_model["name"] == model["name"]:
                existing_model.update(model)
                return
        self.models.append(model)

    def apply_changes(self):
        # No interaction with Gazebo/Isaac Sim or SDF/USD directly
        pass

    def cleanup(self):
        pass