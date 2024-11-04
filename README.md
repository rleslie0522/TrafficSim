## Build Instructions
1. Ensure you are in your ROS 2 Workspace - e.g. `cd ~/ros2_ws`
2. Type `colcon build --packages-select trafficsim`
3. Type `source install/local_setup.bash`
4. Type `ros2 launch trafficsim traffic_launch.py`

## Resolving Pyrobosim Errors
### No module named transforms3d
1. Type `sudo apt install python3-transforms3d`
2. Type `sudo apt install python3-shapely`
3. Type `pip install trimesh adjustText astar Pyside6 --break-system-packages`
4. Attempt the steps in **Build Instructions**.