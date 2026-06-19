import sys
import os
sys.path.append(os.path.join(os.getcwd(), 'code', 'classes'))
sys.path.append(os.path.join(os.getcwd(), 'code'))

from world_manager import WorldManager
from app_state import AppState

wm = WorldManager(simulation="ignition", version="fortress")
wm.create_new_world("test_world")
wm.add_model({
    "name": "test_wall",
    "type": "wall",
    "properties": {
        "start": [0, 0],
        "end": [2, 0],
        "width": 0.2,
        "height": 2.0,
        "color": "Red"
    }
})

class DummyApp:
    def __init__(self):
        self.world_manager = wm
        self.preview_process = None
    def status(self, msg, type):
        print(f"STATUS [{type}]: {msg}")

app = AppState()
app.world_manager = wm
app.launch_preview(use_ros_launch=False)
