## Dependencies

1. This project is build for Ubuntu Linux - Noble Numbat (24.04)
2. [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) - Build according to docs
3. [Pyrobosim](https://pyrobosim.readthedocs.io/en/latest/setup.html#local-setup) - Build according to docs
4. Type `source ./setup/source_pyrobosim.bash` to source and enter the python virtual environment before any steps below

# :warning: IMPORTANT - Read below:

Cloning this folder will create a folder named `CS4048_GroupProject` on your system. If this happens, you must type the following to correct the name of the folder to match the package name:

1. `cd path/to/CS4048_GroupProject/..` or `cd parent/path/of/CS4048_GroupProject`
2. `mv CS4048_GroupProject/ trafficsim/`
3. `mv trafficsim/ path/to/ros2_ws/src`

## Installing requirements

1. `pip install -r path/to/ros2_ws/src/trafficsim/core/requirements.txt`

## Build & Launch Instructions

1. `cd path/to/ros2_ws/src/trafficsim`
2. `bash build_and_launch.bash`

## Resolving Errors - No module named 'py-xxxx'

1. `pip install xxxx`
2. **ONLY if failed** `sudo apt install python3-xxxx`

## Package Contents

There are two folders within this repository - each contain a separate package:

1. `core` contains the `trafficsim` package.
2. `trafficsim_interfaces` contains the custom interfaces used to communicate with TrafficSim package.

## Operation

### Demo Path Planner

1. type `ros2 action send_goal /execute_train_route trafficsim_interfaces/action/ExecuteTrainRoute "{train_id: 'Scotrail_170401', destination: 'Edinburgh_Waverley', stops: ['Leuchars', 'Kirkcaldy', 'Inverkeithing', 'Haymarket']}"`

### Freight Spawner

To begin spawning freight:

```bash
ros2 topic pub /toggle_freight_spawning trafficsim_interfaces/msg/ToggleFreightSpawning '{enabled: true}'
```

To stop spawning freight:

```bash
ros2 topic pub /toggle_freight_spawning trafficsim_interfaces/msg/ToggleFreightSpawning '{enabled: false}'
```
