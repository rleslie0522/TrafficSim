#!/usr/bin/env python3


# ========================================================================================
#
# NAME:         pathing.py
# DESCRIPTION:  Responsible for generating an A* Search Path for train routing.
# AUTHORS:      CALVIN EARNSHAW, FAVOUR JAM, KACPER KOMNATA, REBEKAH LESLIE, ANDREAS MAITA
#
# ========================================================================================


# ========================================================================================
#
# IMPORT DEPENDENCIES
#
# ========================================================================================

import networkx as nx
from typing import Optional
from pyrobosim.utils.pose import Pose
import numpy as np


# ========================================================================================
#
# GLOBAL DEFINITIONS
#
# ========================================================================================

SPEED = 10.0


# ----------------------------------------------------------------------------------------
#
# NAME:         squared_euclidian_dist_between_stations()
# DESCRIPTION:  Returns the Euclidean Distance between two stations.
# PARAMETERS:   station1, station2
# RETURNS:      float - a floating point representation of the Euclidean Distance.
#
# ----------------------------------------------------------------------------------------

def squared_euclidian_dist_between_stations(station1, station2):
    return ((station1.pose.x - station2.pose.x)**2 + (station1.pose.y - station2.pose.y)**2)


# ========================================================================================
#
# CLASS DEFINITIONS
#
# ========================================================================================


# ----------------------------------------------------------------------------------------
#
# NAME:         Station
# DESCRIPTION:  A representation of a Station and its x,y pose within Pyrobosim.
# PARAMETERS:   x, y, name
#
# ----------------------------------------------------------------------------------------

class Station:
    def __init__(self, x: float, y: float, name: str):
        self.pose = Pose(x, y, 0)
        self.name = name

    def __repr__(self):
        return f"{self.name} ({self.pose})"


# ----------------------------------------------------------------------------------------
#
# NAME:         Connection
# DESCRIPTION:  TODO
# PARAMETERS:   none
#
# ----------------------------------------------------------------------------------------

class Connection:
    def __init__(self):
        self.claimed = False


# ----------------------------------------------------------------------------------------
#
# NAME:         StationGraph
# DESCRIPTION:  A graph representation of a train station and its connection to other
#               stations.
# PARAMETERS:   - stations_with_coords: a dictionary of station-coordinate pairs,
#               - connections: a dictionary of station-coordinate pairs.
#
# ----------------------------------------------------------------------------------------

class StationGraph:

    # ------------------------------------------------------------------------------------
    #
    # Class Constructor
    #
    # ------------------------------------------------------------------------------------
    def __init__(self, stations_with_coords: dict[str, tuple[str, str]], connections: dict[str, tuple[str, str]]):
        station_dict = {}
        station_graph = nx.MultiDiGraph()

        for station_name, (x, y)  in stations_with_coords.items():
            new_station = Station(float(x), float(y), station_name)
            station_graph.add_node(new_station)
            station_dict[station_name] = new_station

        for first_station_name, second_station_name in connections.values():
            distance = squared_euclidian_dist_between_stations(station_dict[first_station_name], station_dict[second_station_name])
            station_graph.add_edge(station_dict[first_station_name], station_dict[second_station_name], object=Connection(), weight=distance)
            station_graph.add_edge(station_dict[second_station_name], station_dict[first_station_name], object=Connection(), weight=distance)

        self.station_graph = station_graph
        self.station_dict = station_dict

    # ------------------------------------------------------------------------------------
    #
    # NAME:         get_path_between_nodes
    # DESCRIPTION:  Returns the A* Path between a starting and ending node using NetworkX.
    # PARAMETERS:   - start_node: a Station object containing name and x,y Pose.
    #               - end_node: a Station object containing name and x,y Pose.
    # RETURNS:      - list of Station objects
    #
    # ------------------------------------------------------------------------------------
    def get_path_between_nodes(self, start_node: Station, end_node: Station) -> list[Station]:
        return nx.astar_path(self.station_graph, start_node, end_node, squared_euclidian_dist_between_stations, weight='weight')

    # ------------------------------------------------------------------------------------
    #
    # NAME:         get_connection_between_nodes
    # DESCRIPTION:  TODO
    # PARAMETERS:   - start_node: a Station object containing name and x,y Pose.
    #               - end_node: a Station object containing name and x,y Pose.
    # RETURNS:      - a Connection object TODO
    #
    # ------------------------------------------------------------------------------------
    def get_connection_between_nodes(self, start_node: Station, end_node: Station) -> Connection:
        return self.station_graph[start_node][end_node][0]['object']

    # ------------------------------------------------------------------------------------
    #
    # NAME:         get_node
    # DESCRIPTION:  Returns a node within the StationGraph class.
    # PARAMETERS:   - name: a String containing the name of the node to be found.
    # RETURNS:      - a Station node.
    #
    # ------------------------------------------------------------------------------------
    def get_node(self, name: str) -> Optional[Station]:
        return self.station_dict.get(name)


# ----------------------------------------------------------------------------------------
#
# NAME:         PathFollower
# DESCRIPTION:  TODO
# PARAMETERS:   - start: a Pose object
#               - end: a Pose object
#               - speed_mult: a Float value determining the speed of the path execution
#
# ----------------------------------------------------------------------------------------

class PathFollower:

    # ------------------------------------------------------------------------------------
    #
    # Class Constructor
    #
    # ------------------------------------------------------------------------------------
    def __init__(self, start: Pose, end: Pose, speed_mult: float = 1.0):
        self.current_pos = start.get_translation()

        move_vector = end.get_translation() - self.current_pos
        self.total_distance = np.linalg.norm(move_vector)
        self.direction = move_vector / self.total_distance if self.total_distance > 0 else np.zeros(3)
        self.distance_traveled = 0.0
        self.speed_mult = speed_mult

    # ------------------------------------------------------------------------------------
    #
    # NAME:         get_next_pose
    # DESCRIPTION:  TODO
    # PARAMETERS:   - dt - a Float object
    # RETURNS:      - a Pose object
    #
    # ------------------------------------------------------------------------------------
    def get_next_pose(self, dt: float) -> Pose:
        dist_to_move = max(min(SPEED * self.speed_mult * dt, self.total_distance - self.distance_traveled), 0)
        self.distance_traveled += dist_to_move
        self.current_pos += self.direction * dist_to_move

        return Pose(self.current_pos[0], self.current_pos[1], self.current_pos[2])

    def __repr__(self) -> str:
        return f"PathFollower(current_pos={self.current_pos}, speed_mult={self.speed_mult}, total_distance={self.total_distance}, direction={self.direction}, distance_traveled={self.distance_traveled})"
