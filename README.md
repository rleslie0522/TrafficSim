Cloning this folder will create a folder named `TrafficSim` on your system. If this happens, you must type the following to correct the name of the folder to match the package name:

1. `cd path/to/TrafficSim/..` or `cd parent/path/of/TrafficSim`
2. `mv TrafficSim/ trafficsim/`
3. `mv trafficsim/ path/to/ros2_ws/src`

&nbsp;

---

&nbsp;

# TrafficSim - A package for simulating the Scottish Rail Network in ROS2.

## Dependencies

1. This project is build for Ubuntu Linux - Noble Numbat (24.04)
2. [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) - Build according to docs
3. [Pyrobosim](https://pyrobosim.readthedocs.io/en/latest/setup.html#local-setup) - Build according to docs
4. Type `source ./setup/source_pyrobosim.bash` to source and enter the python virtual environment before any steps below

## Installing requirements

1. `pip install -r path/to/ros2_ws/src/trafficsim/core/requirements.txt`

## Build & Launch Instructions

1. `cd path/to/ros2_ws/src/trafficsim`
2. `bash build_and_launch.bash`

## Resolving Errors - No module named 'py-xxxx'

1. `pip install xxxx`
2. **ONLY if failed** `sudo apt install python3-xxxx`

## Package Contents

There are three folders within this repository - each contain a separate package:

1. `core` contains the `trafficsim` package.
2. `trafficsim_interfaces` contains the custom interfaces used to communicate with TrafficSim package.
3. `station_dataset` contains the station coordinates and CRS codes (in `csv` format), as well as a Python script, `dataset_to_json.py` to generate rail lines and stations based on an internal dictionary, `lines`.

## Actions

### Moving a Train to a Destination (no stops)
```bash
ros2 action send_goal <RobotName>/route_train trafficsim_interfaces/action/RouteTrain "{destination: 'Station_Name'}"
```

### Running Live Timetable on a Train
```bash
ros2 action send_goal <RobotName>/follow_service_timetable trafficsim_interfaces/action/ServiceRoute
```

## Services

### Request Next Service
```bash
ros2 service call /RailTrafficScheduler/request_next_departure trafficsim_interfaces/srv/CRSDepartureLookup "{origin: 'Station_Name', lookup_only: True}"
```
returns a response using the `CRSDepartureLookup` interface, similar to:
```bash
trafficsim_interfaces.srv.CRSDepartureLookup_Response(api_status_code='200', service_uid='P78998', destination='Glasgow Queen Street', origin_start_time='0530', destination_arrival_time='0831', headcode='1T10', atoc_name='ScotRail')
```

### Request All Services
```bash
ros2 service call /RailTrafficScheduler/request_all_departures trafficsim_interfaces/srv/CRSAllDepartures "{origin: 'Station_Name', year: 'yyyy', month: 'mm', date: 'dd'}"
```
returns a response using the `CRSAllDepartures` interface, similar to:
```bash
trafficsim_interfaces.srv.CRSAllDepartures_Response(api_status_code='200', service_uid_list=['P78998', 'P78058', 'P78281', 'P79383', 'P79001', 'P79005', 'C46737', 'C48579', 'P78287', ..., 'P79411'])
```

### Request Service Details
```bash
ros2 service call /RailTrafficScheduler/request_service_details trafficsim_interfaces/APIRailServiceLookup "{service_uid: 'P78077', year: '2024', month: '12', date: '06'}"
```
returns a response using the `APIRailServiceLookup` interface, similar to:
```bash
trafficsim_interfaces.srv.APIRailServiceLookup_Response(api_status_code='200', origin='Aberdeen', destination='Edinburgh', stops=['Stonehaven', 'Laurencekirk', 'Montrose', 'Arbroath', 'Carnoustie', 'Dundee', 'Leuchars', 'Haymarket', 'Edinburgh'])
```


## Topics

### Freight Spawner

To begin spawning freight:

```bash
ros2 topic pub /toggle_freight_spawning trafficsim_interfaces/msg/ToggleFreightSpawning '{enabled: true}'
```

To stop spawning freight:

```bash
ros2 topic pub /toggle_freight_spawning trafficsim_interfaces/msg/ToggleFreightSpawning '{enabled: false}'
```
