#!/usr/bin/env python
"""
| File: people.py
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to build an app that makes use of the Pegasus API to run a simulation
| where people move around in the world.
"""

# Imports to start Isaac Sim from this script
import carb

from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World
from isaacsim.core.utils.extensions import enable_extension

# Enable/disable ROS bridge extensions to keep only ROS2 Bridge
enable_extension("isaacsim.ros2.bridge")

# Update the simulation app with the new extensions
simulation_app.update()

# -------------------------------------------------------------------------------------------------
# These lines are needed to restart the USD stage and make sure that the people extension is loaded
# -------------------------------------------------------------------------------------------------
import omni.usd
omni.usd.get_context().new_stage()

import numpy as np

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.people.person import Person
from pegasus.simulator.logic.people.person_controller import PersonController
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig

# Example controller class that make a person move in a circle around the origin of the world
# Note: You could create a different controller with a different behaviour. For instance, you could:
# 1. read the keyboard input to move the person around the world.
# 2. read the target position from a ros topic,
# 3. read the target position from a file,
# 4. etc.
class CirclePersonController(PersonController):

    def __init__(self):
        super().__init__()

        self._radius = 5.0
        self.gamma = 0.0
        self.gamma_dot = 0.3
        
    def update(self, dt: float):

        # Update the reference position for the person to track
        self.gamma += self.gamma_dot * dt
        
        # Set the target position for the person to track
        self._person.update_target_position([self._radius * np.cos(self.gamma), self._radius * np.sin(self.gamma), 0.0])
        

# Auxiliary scipy and numpy modules
from scipy.spatial.transform import Rotation

# -------------------------------------------------------------------------------------------------
# Define the PegasusApp class where the simulation will be run
# -------------------------------------------------------------------------------------------------
class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics,
        # temporary memory etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by Isaac Sim (Gazebo-like)
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Default Environment"])

        # Create a person and attach a controller to it
        person1 = Person("person1", init_pos=[5.0, 0.0, 0.0])
        person1.update_controller(CirclePersonController())

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Auxiliary variable to stop the simulation
        self.stop_sim = False

    def run(self):
        """
        Method that implements the main simulation loop.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:
            # Update the UI of the app and perform the physics step
            self.world.step(render=True)
        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():

    # Instantiate the PegasusApp
    pg_app = PegasusApp()

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()
