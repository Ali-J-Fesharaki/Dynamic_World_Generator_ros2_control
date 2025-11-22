#!/bin/bash
ign gazebo /home/ajf/map_maker_prgoram/Dynamic_World_Generator/worlds/gazebo/fortress/maze.sdf &
sleep 2
python3 /home/ajf/map_maker_prgoram/Dynamic_World_Generator/worlds/gazebo/fortress/move_code/maze_moveObstacles.py &
wait
