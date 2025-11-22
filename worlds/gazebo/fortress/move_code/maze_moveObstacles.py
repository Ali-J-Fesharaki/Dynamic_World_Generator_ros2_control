#!/usr/bin/env python3
import time
import random
import math

import subprocess

prefix = "ign"
reqtype_prefix = "ignition.msgs"
world_name = "maze"

def set_pose(model_name, x, y, z):
    request_str = f'name: "{model_name}", position {{ x: {x} y: {y} z: {z} }}, orientation {{ w: 1 }}'
    cmd = [prefix, "service", "-s", f"/world/{world_name}/set_pose", "--reqtype", f"{reqtype_prefix}.Pose", "--reptype", f"{reqtype_prefix}.Boolean", "--timeout", "500", "--req", request_str]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        return False
    return True

motions = {}
states = {}
motions["box_5"] = {'type': 'linear', 'velocity': 1.0, 'std': 0.1, 'path': [(0.6, -0.0), (0.7, 1.4)]}
states["box_5"] = {'current_pos': [0.6, -0.0], 'direction': 1, 'start': [0.6, -0.0], 'end': [0.7, 1.4], 'z': 0.1}
motions["box_6"] = {'type': 'linear', 'velocity': 1.0, 'std': 0.1, 'path': [(-0.9, 1.0), (-1.0, 2.3)]}
states["box_6"] = {'current_pos': [-0.9, 1.0], 'direction': 1, 'start': [-0.9, 1.0], 'end': [-1.0, 2.3], 'z': 0.1}
motions["box_7"] = {'type': 'linear', 'velocity': 1.0, 'std': 0.1, 'path': [(-0.9, -0.8), (-0.9, 0.5)]}
states["box_7"] = {'current_pos': [-0.9, -0.8], 'direction': 1, 'start': [-0.9, -0.8], 'end': [-0.9, 0.5], 'z': 0.1}

dt = 0.005
linear_dt = 0.001
while True:
    try:
        for model_name, motion in motions.items():
            state = states[model_name]
            velocity = max(min(random.gauss(motion["velocity"], motion["std"]), motion["velocity"] * 2), 0)
            delta = velocity * (linear_dt if motion["type"] == "linear" else dt)
            if motion["type"] == "linear":
                start = state["start"]
                end = state["end"]
                dx = end[0] - start[0]
                dy = end[1] - start[1]
                length = math.sqrt(dx**2 + dy**2)
                if length < 0.001: continue
                unit_x = dx / length
                unit_y = dy / length
                new_x = state["current_pos"][0] + delta * state["direction"] * unit_x
                new_y = state["current_pos"][1] + delta * state["direction"] * unit_y
                t = ((new_x - start[0]) * dx + (new_y - start[1]) * dy) / (length**2)
                if t > 1:
                    new_x = end[0]
                    new_y = end[1]
                    state["direction"] = -state["direction"]
                elif t < 0:
                    new_x = start[0]
                    new_y = start[1]
                    state["direction"] = -state["direction"]
                else:
                    new_x = start[0] + t * dx
                    new_y = start[1] + t * dy
                state["current_pos"] = [new_x, new_y]
                if not set_pose(model_name, new_x, new_y, state["z"]):
                    exit(1)
            elif motion["type"] == "elliptical":
                delta_theta = velocity / (2 * math.pi * motion["semi_major"]) * 2 * math.pi * dt
                state["theta"] += delta_theta
                theta = state["theta"]
                x = state["center"][0] + motion["semi_major"] * math.cos(theta) * math.cos(motion["angle"]) - motion["semi_minor"] * math.sin(theta) * math.sin(motion["angle"])
                y = state["center"][1] + motion["semi_major"] * math.cos(theta) * math.sin(motion["angle"]) + motion["semi_minor"] * math.sin(theta) * math.cos(motion["angle"])
                if not set_pose(model_name, x, y, state["z"]):
                    exit(1)
            elif motion["type"] == "polygon":
                path = state["path"]
                start = path[state["current_segment"]]
                end = path[(state["current_segment"] + 1) % len(path)]
                dx = end[0] - start[0]
                dy = end[1] - start[1]
                length = math.sqrt(dx**2 + dy**2)
                if length < 0.001: continue
                state["t"] += delta / length
                while state["t"] >= 1:
                    state["t"] -= 1
                    state["current_segment"] = (state["current_segment"] + 1) % len(path)
                start = path[state["current_segment"]]
                end = path[(state["current_segment"] + 1) % len(path)]
                dx = end[0] - start[0]
                dy = end[1] - start[1]
                x = start[0] + state["t"] * dx
                y = start[1] + state["t"] * dy
                if not set_pose(model_name, x, y, state["z"]):
                    exit(1)
        time.sleep(linear_dt if motion["type"] == "linear" else dt)
    except KeyboardInterrupt:
        exit(0)
