parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path/../../"
cp ./src/trafficsim/core/trafficsim/prs_worldinit.py ./install/pyrobosim_ros/lib/pyrobosim_ros
chmod +x ./install/pyrobosim_ros/lib/pyrobosim_ros/prs_worldinit.py
colcon build --packages-select pyrobosim pyrobosim_ros trafficsim trafficsim_interfaces
source ./install/local_setup.bash
ros2 launch trafficsim traffic_launch.py
