import sys
import os
sys.path.append(os.path.join(os.getcwd(), 'code', 'classes'))
sys.path.append(os.path.join(os.getcwd(), 'code'))

from world_manager import WorldManager

wm = WorldManager(simulation="ignition", version="fortress")
wm.world_name = "test_world"
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

wm.sync_static_models_to_running_gazebo()
print("Done!")
