## Dependencies

1. This project is build for Ubuntu Linux - Noble Numbat (24.04)
2. [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) - Build according to docs
3. [Pyrobosim](https://pyrobosim.readthedocs.io/en/latest/setup.html#local-setup) - Build according to docs
4. Type `source ./setup/source_pyrobosim.bash` to source the virtual environment before any steps below

# :warning: IMPORTANT - Read below:

Cloning this folder will create a folder named `CS4048_GroupProject` on your system. If this happens, you must type the following to correct the name of the folder to match the package name:

1. `cd /path/to/parent/of/CS4048_GroupProject`
2. `mv CS4048_GroupProject/ trafficsim/`
3. `mv trafficsim/ path/to/ros2_ws/src`

## Build Instructions

```sh
bash build_and_launch.bash
```

## Resolving Pyrobosim Errors

### No module named transforms3d

1. Type `sudo apt install python3-transforms3d`
2. Type `sudo apt install python3-shapely`
3. Type `pip install trimesh adjustText astar Pyside6 --break-system-packages`
4. Attempt the steps in **Build Instructions**.

### No module named 'py-xxxx'

```sh
pip install -r requirements.txt
```

## Package Contents
There are two folders within this repository - each contain a separate package:
1. `core` contains the `trafficsim` package.
2. `trafficsim_interfaces` contains the custom interfaces used to communicate with TrafficSim package.