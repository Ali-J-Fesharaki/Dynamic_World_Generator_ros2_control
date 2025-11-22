import os
from utils.config import PROJECT_ROOT, WORLDS_GAZEBO_DIR

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
        self.world_path = os.path.join(WORLDS_GAZEBO_DIR, self.version, f"{world_name}.sdf")
        self.models = []

    def load_world(self, world_name):
        # Load an existing world (read-only for context)
        self.world_name = world_name
        self.world_path = os.path.join(WORLDS_GAZEBO_DIR, self.version, f"{world_name}.sdf")
        self.models = []
        # We no longer parse SDF. Models are managed in memory for the session.

    def add_model(self, model):
        # Add or update a model in the world
        for existing_model in self.models:
            if existing_model["name"] == model["name"]:
                existing_model.update(model)
                return
        self.models.append(model)

    def apply_changes(self):
        # No interaction with Gazebo or SDF
        pass

    def cleanup(self):
        pass