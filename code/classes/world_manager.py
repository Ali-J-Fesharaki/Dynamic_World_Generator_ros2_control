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
        # We write walls directly to the .sdf file.
        # Obstacles are handled separately via obstacles.yaml by the launch file.
        if not self.world_name:
            return
            
        import shutil
        import math
        from xml.etree import ElementTree as ET
        
        # 1. Read the empty world template
        empty = os.path.join(WORLDS_GAZEBO_DIR, self.version, "empty_world.sdf")
        if not os.path.exists(empty):
            return
            
        try:
            tree = ET.parse(empty)
            world_elem = tree.getroot().find("world")
            if world_elem is not None:
                world_elem.set("name", self.world_name)
                
                # 2. Append all walls as models
                for m in self.models:
                    if m["type"] == "wall" and m.get("status") != "removed":
                        props = m["properties"]
                        sx, sy = props["start"]
                        ex, ey = props["end"]
                        w = props["width"]
                        h = props["height"]
                        
                        cx = (sx + ex) / 2.0
                        cy = (sy + ey) / 2.0
                        cz = h / 2.0
                        length = math.hypot(ex - sx, ey - sy)
                        yaw = math.atan2(ey - sy, ex - sx)
                        
                        model_elem = ET.SubElement(world_elem, "model", name=m["name"])
                        static_elem = ET.SubElement(model_elem, "static")
                        static_elem.text = "true"
                        pose_elem = ET.SubElement(model_elem, "pose")
                        pose_elem.text = f"{cx:.3f} {cy:.3f} {cz:.3f} 0 0 {yaw:.3f}"
                        
                        link_elem = ET.SubElement(model_elem, "link", name="link")
                        
                        for geom_type in ["collision", "visual"]:
                            comp_elem = ET.SubElement(link_elem, geom_type, name=geom_type)
                            geom = ET.SubElement(comp_elem, "geometry")
                            box = ET.SubElement(geom, "box")
                            size = ET.SubElement(box, "size")
                            size.text = f"{length:.3f} {w:.3f} {h:.3f}"
                            
                            if geom_type == "visual":
                                mat = ET.SubElement(comp_elem, "material")
                                amb = ET.SubElement(mat, "ambient")
                                amb.text = "0.7 0.7 0.7 1"
                                dif = ET.SubElement(mat, "diffuse")
                                dif.text = "0.7 0.7 0.7 1"
                                
            # 3. Save to src directory
            src_path = os.path.join(PROJECT_ROOT, "code", "control_ws", "src",
                                    "dynamic_obstacle_gz_spawning", "worlds", f"{self.world_name}.sdf")
            os.makedirs(os.path.dirname(src_path), exist_ok=True)
            tree.write(src_path, encoding="utf-8", xml_declaration=True)
            
            # 4. Save to install directory (so Gazebo can run it immediately without rebuild)
            install_path = os.path.join(PROJECT_ROOT, "code", "control_ws", "install",
                                        "dynamic_obstacle_gz_spawning", "share",
                                        "dynamic_obstacle_gz_spawning", "worlds", f"{self.world_name}.sdf")
            os.makedirs(os.path.dirname(install_path), exist_ok=True)
            shutil.copyfile(src_path, install_path)
            
        except Exception as e:
            print(f"Error applying SDF changes: {e}")

    def cleanup(self):
        pass