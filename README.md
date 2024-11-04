## Build Instructions
1. Ensure you are in your ROS 2 Workspace - e.g. `cd ~/ros2_ws`
2. Type `colcon build --packages-select trafficsim`
3. Type `source install/local_setup.bash`
4. Type `ros2 launch trafficsim traffic_launch.py`