# CS4048 - Group 3

## Setup Instructions
1. Clone this repository into a folder entitled `CS4048_GroupProject` within your ROS2 `/src` directory - e.g. `~/ros2_ws/src/CS4048_GroupProject`.

## Build Instructions
1. Type `cd ~/ros2_ws`
2. Type `colcon build --packages-select CS4048_GroupProject`
3. Type `source install/local_setup.bash`
4. Type `ros2 launch CS4048_GroupProject traffic_launch.py`