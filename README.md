cd ## Dependencies
1. This project is build for Ubuntu Linux - Noble Numbat (24.04)
2. [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) - Build according to docs
3. [Pyrobosim](https://pyrobosim.readthedocs.io/en/latest/setup.html#local-setup) - Build according to docs
  1. Type `source ./setup/source_pyrobosim.bash` to source the virtual environment before any steps below

# :warning: IMPORTANT - Read below:
Cloning this folder will create a folder named `CS4048_GroupProject` on your system. If this happens, you must type the following to correct the name of the folder to match the package name:
1. `cd /path/to/parent/of/CS4048_GroupProject`
2. `mv CS4048_GroupProject/ trafficsim/`
3. `mv trafficsim/ path/to/ros2_ws/src`

## Build Instructions
1. Ensure you are in your ROS 2 Workspace - e.g. `cd ~/ros2_ws`
2. Type `cp ~/ros2_ws/src/trafficsim/trafficsim/prs_worldinit.py ~/ros2_ws/install/pyrobosim_ros/lib/pyrobosim_ros` (**:warning: WARNING :warning:** - you may need to alter the destination path based on the location of your Pyrobosim installation directory)
3. Type `chmod +x ~/ros2_ws/install/pyrobosim_ros/lib/pyrobosim_ros/prs_worldinit.py`
3. Type `colcon build --packages-select pyrobosim pyrobosim_ros trafficsim`
4. Type `source install/local_setup.bash`
5. Type `ros2 launch trafficsim traffic_launch.py`

## Resolving Pyrobosim Errors
### No module named transforms3d
1. Type `sudo apt install python3-transforms3d`
2. Type `sudo apt install python3-shapely`
3. Type `pip install trimesh adjustText astar Pyside6 --break-system-packages`
4. Attempt the steps in **Build Instructions**.