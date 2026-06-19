import os
from utils.config import PROJECT_ROOT, WORLDS_GAZEBO_DIR

class WorldManager:
    def __init__(self, simulation, version):
        # Initialize world manager with simulation and version
        self.simulation = simulation
        self.sim_type = simulation
        self.version = version
        self.world_name = None
        self.world_path = None
        self.models = []
        self._synced_models_hash = {}
        self._already_removed_models = set()
        self.map_path = None # Path to the background map image (PGM)

    def create_new_world(self, world_name):
        # Create a new world state (no file creation)
        self.world_name = world_name
        self.world_path = os.path.join(WORLDS_GAZEBO_DIR, self.version, f"{world_name}.sdf")
        self.models = []
        self._synced_models_hash = {}
        self._already_removed_models = set()

    def load_world(self, world_name):
        # Load an existing world (read-only for context)
        self.world_name = world_name
        self.world_path = os.path.join(WORLDS_GAZEBO_DIR, self.version, f"{world_name}.sdf")
        self.models = []
        self._synced_models_hash = {}
        self._already_removed_models = set()
        # We no longer parse SDF. Models are managed in memory for the session.

    def add_model(self, model):
        # Add or update a model in the world
        for existing_model in self.models:
            if existing_model["name"] == model["name"]:
                existing_model.update(model)
                return
        self.models.append(model)

    def _add_model_to_elem(self, m, parent_elem):
        import xml.etree.ElementTree as ET
        from utils.color_utils import get_color
        
        props = m["properties"]
        color_name = props.get("color", "Gray")
        r, g, b = get_color(color_name)

        if m["type"] == "wall":
            sx, sy = props["start"]
            ex, ey = props["end"]
            w = props["width"]
            h = props["height"]
            
            import math
            length = math.hypot(ex - sx, ey - sy)
            cx = (sx + ex) / 2.0
            cy = (sy + ey) / 2.0
            yaw = math.atan2(ey - sy, ex - sx)
            
            model_elem = ET.SubElement(parent_elem, "model", name=m["name"])
            static_elem = ET.SubElement(model_elem, "static")
            static_elem.text = "true"
            pose_elem = ET.SubElement(model_elem, "pose")
            pose_elem.text = f"{cx:.3f} {cy:.3f} {h/2.0:.3f} 0 0 {yaw:.3f}"
            
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
                    amb.text = f"{r} {g} {b} 1"
                    dif = ET.SubElement(mat, "diffuse")
                    dif.text = f"{r} {g} {b} 1"

        elif m["type"] in ("box", "cylinder", "sphere"):
            # If it has motion, it's spawned by ROS2 via yaml, not SDF
            if "motion" in props:
                return

            # Otherwise it's static, add to SDF
            cx, cy, cz = props["position"]
            model_elem = ET.SubElement(parent_elem, "model", name=m["name"])
            static_elem = ET.SubElement(model_elem, "static")
            static_elem.text = "true"
            pose_elem = ET.SubElement(model_elem, "pose")
            pose_elem.text = f"{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0"
            
            link_elem = ET.SubElement(model_elem, "link", name="link")
            for geom_type in ["collision", "visual"]:
                comp_elem = ET.SubElement(link_elem, geom_type, name=geom_type)
                geom = ET.SubElement(comp_elem, "geometry")
                
                if m["type"] == "box":
                    w, l, hi = props["size"]
                    shape = ET.SubElement(geom, "box")
                    size = ET.SubElement(shape, "size")
                    size.text = f"{w:.3f} {l:.3f} {hi:.3f}"
                elif m["type"] == "cylinder":
                    rad, hi = props["size"]
                    shape = ET.SubElement(geom, "cylinder")
                    r_el = ET.SubElement(shape, "radius")
                    r_el.text = f"{rad:.3f}"
                    l_el = ET.SubElement(shape, "length")
                    l_el.text = f"{hi:.3f}"
                elif m["type"] == "sphere":
                    rad = props["size"][0]
                    shape = ET.SubElement(geom, "sphere")
                    r_el = ET.SubElement(shape, "radius")
                    r_el.text = f"{rad:.3f}"
                    
                if geom_type == "visual":
                    mat = ET.SubElement(comp_elem, "material")
                    amb = ET.SubElement(mat, "ambient")
                    amb.text = f"{r} {g} {b} 1"
                    dif = ET.SubElement(mat, "diffuse")
                    dif.text = f"{r} {g} {b} 1"

    def sync_static_models_to_running_gazebo(self):
        if self.sim_type == "isaacsim":
            return
            
        import xml.etree.ElementTree as ET
        import subprocess
        import os
        import tempfile
        import shutil
        import hashlib
        import json
        
        if shutil.which("ign"):
            cmd_prefix = "ign"
            msg_prefix = "ign_msgs"
        else:
            cmd_prefix = "gz"
            msg_prefix = "gz.msgs"
            
        world = self.world_name if self.world_name else "multi_obstacle_world"
        temp_dir = tempfile.gettempdir()
        
        to_remove = []
        to_create = []
        
        for m in self.models:
            is_dynamic = m["type"] in ("box", "cylinder", "sphere") and "motion" in m.get("properties", {})
                
            name = m["name"]
            status = m.get("status")
            
            if status == "removed":
                if name not in self._already_removed_models:
                    to_remove.append(name)
                    self._already_removed_models.add(name)
                if name in self._synced_models_hash:
                    del self._synced_models_hash[name]
                continue
            
            prop_str = json.dumps(m.get("properties", {}), sort_keys=True)
            m_hash = hashlib.md5(prop_str.encode()).hexdigest()
            
            if name not in self._synced_models_hash:
                if not is_dynamic:
                    to_create.append(m)
                self._synced_models_hash[name] = m_hash
            elif self._synced_models_hash[name] != m_hash:
                to_remove.append(name)
                if not is_dynamic:
                    to_create.append(m)
                self._synced_models_hash[name] = m_hash
                
        for name in to_remove:
            cmd = [cmd_prefix, "service", "-s", f"/world/{world}/remove",
                   "--reqtype", f"{msg_prefix}.Entity", "--reptype", f"{msg_prefix}.Boolean",
                   "--timeout", "1000", "--req", f"name: '{name}' type: MODEL"]
            subprocess.run(cmd, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
            
        for m in to_create:
            sdf_root = ET.Element("sdf", version="1.6")
            self._add_model_to_elem(m, sdf_root)
            sdf_str = ET.tostring(sdf_root, encoding="unicode")
            
            temp_sdf = os.path.join(temp_dir, f"{m['name']}.sdf")
            with open(temp_sdf, "w") as f:
                f.write(sdf_str)
                
            cmd = [cmd_prefix, "service", "-s", f"/world/{world}/create",
                   "--reqtype", f"{msg_prefix}.EntityFactory", "--reptype", f"{msg_prefix}.Boolean",
                   "--timeout", "2000", "--req", f"sdf_filename: '{temp_sdf}' name: '{m['name']}'"]
            subprocess.run(cmd, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)

    def apply_changes(self):
        # We write walls directly to the .sdf file.
        # Obstacles are handled separately via obstacles.yaml by the launch file.
        if not self.world_name:
            return
            
        if self.sim_type == "isaacsim":
            from classes.usd_exporter import export_to_usda
            import yaml
            
            export_path = os.path.join(PROJECT_ROOT, "worlds", "isaacsim", f"{self.world_name}.usd")
            os.makedirs(os.path.dirname(export_path), exist_ok=True)
            export_to_usda(self.models, export_path)
            
            # Export obstacles.yaml for isaac sim spawning script
            data = {"obstacles": []}
            for m in self.models:
                if m.get("status")=="removed" or m["type"]=="wall" or m["type"] not in ("box","cylinder","sphere", "person"):
                    continue
                if "motion" not in m.get("properties", {}) and m["type"] != "person":
                    continue
                
                # Person doesn't use color
                color_val = m["properties"].get("color","gray").lower() if m["type"] != "person" else "none"
                
                o = {"name":m["name"],"type":m["type"],"color":color_val,
                     "enabled":True,"x_pose":float(m["properties"]["position"][0]),
                     "y_pose":float(m["properties"]["position"][1]),
                     "z_pose":float(m["properties"]["position"][2]),
                     "size":list(m["properties"].get("size", [1,1,1]))}
                if "motion" in m["properties"]:
                    mt = m["properties"]["motion"]
                    o["motion"] = {"type":mt["type"],"velocity":float(mt.get("velocity", 1.0)),"std":float(mt.get("std", 0.0))}
                    if "path" in mt:
                        o["motion"]["path"] = [[p[0]-float(m["properties"]["position"][0]),
                                                p[1]-float(m["properties"]["position"][1])] for p in mt["path"]]
                    if mt["type"]=="elliptical":
                        o["motion"]["semi_major"]=float(mt["semi_major"])
                        o["motion"]["semi_minor"]=float(mt["semi_minor"])
                        o["motion"]["angle"]=float(mt["angle"])
                data["obstacles"].append(o)
                
            obs_yaml_path = os.path.join(PROJECT_ROOT, "code", "control_ws", "src", 
                                         "dynamic_obstacle_isaacsim_spawning", "config", "obstacles.yaml")
            os.makedirs(os.path.dirname(obs_yaml_path), exist_ok=True)
            with open(obs_yaml_path, "w") as f:
                yaml.dump(data, f, sort_keys=False)
            return

        import shutil
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
                
                # 2. Append all walls and static obstacles as models
                for m in self.models:
                    if m.get("status") == "removed":
                        continue
                    self._add_model_to_elem(m, world_elem)
                                
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
            
            # 5. Export obstacles.yaml
            import yaml
            data = {"obstacles": []}
            for m in self.models:
                if m.get("status")=="removed" or m["type"]=="wall" or m["type"] not in ("box","cylinder","sphere"):
                    continue
                if "motion" not in m.get("properties", {}):
                    continue
                o = {"name":m["name"],"type":m["type"],"color":m["properties"].get("color","gray").lower(),
                     "enabled":True,"x_pose":float(m["properties"]["position"][0]),
                     "y_pose":float(m["properties"]["position"][1]),
                     "z_pose":float(m["properties"]["position"][2]),"size":list(m["properties"]["size"])}
                if "motion" in m["properties"]:
                    mt = m["properties"]["motion"]
                    o["motion"] = {"type":mt["type"],"velocity":float(mt["velocity"]),"std":float(mt["std"])}
                    if "path" in mt:
                        o["motion"]["path"] = [[p[0]-float(m["properties"]["position"][0]),
                                                p[1]-float(m["properties"]["position"][1])] for p in mt["path"]]
                    if mt["type"]=="elliptical":
                        o["motion"]["semi_major"]=float(mt["semi_major"])
                        o["motion"]["semi_minor"]=float(mt["semi_minor"])
                        o["motion"]["angle"]=float(mt["angle"])
                data["obstacles"].append(o)
                
            obs_yaml_path = os.path.join(PROJECT_ROOT, "code", "control_ws", "src", 
                                         "dynamic_obstacle_gz_spawning", "config", "obstacles.yaml")
            os.makedirs(os.path.dirname(obs_yaml_path), exist_ok=True)
            with open(obs_yaml_path, "w") as f:
                yaml.dump(data, f, sort_keys=False)
            
        except Exception as e:
            print(f"Error applying SDF changes: {e}")

    def cleanup(self):
        pass