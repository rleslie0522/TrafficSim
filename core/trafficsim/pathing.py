#!/usr/bin/env python3


# ========================================================================================
#
# NAME:         pathing.py
# DESCRIPTION:  TODO
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


# ========================================================================================
#
# CLASS DEFINITIONS
#
# ========================================================================================

class Station:
    def __init__(self, x: float, y: float, name: str):
        self.pose = Pose(x, y, 0)
        self.name = name

    def __repr__(self):
        return f"{self.name} ({self.pose})"

def squared_euclidian_dist_between_stations(station1: Station, station2: Station):
    return ((station1.pose.x - station2.pose.x)**2 + (station1.pose.y - station2.pose.y)**2)

class Connection:
    def __init__(self):
        self.claimed = False

class StationGraph:
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

    def get_path_between_nodes(self, start_node: Station, end_node: Station) -> list[Station]:
        return nx.astar_path(self.station_graph, start_node, end_node, squared_euclidian_dist_between_stations, weight='weight')

    def get_connection_between_nodes(self, start_node: Station, end_node: Station) -> Connection:
        return self.station_graph[start_node][end_node][0]['object']

    def get_node(self, name: str) -> Optional[Station]:
        return self.station_dict.get(name)

class PathFollower:
    def __init__(self, start: Pose, end: Pose, speed_mult: float = 1.0):
        self.current_pos = start.get_translation()

        move_vector = end.get_translation() - self.current_pos
        self.total_distance = np.linalg.norm(move_vector)
        self.direction = move_vector / self.total_distance if self.total_distance > 0 else np.zeros(3)
        self.distance_traveled = 0.0
        self.speed_mult = speed_mult

    def get_next_pose(self, dt: float) -> Pose:
        dist_to_move = max(min(SPEED * self.speed_mult * dt, self.total_distance - self.distance_traveled), 0)
        self.distance_traveled += dist_to_move
        self.current_pos += self.direction * dist_to_move

        return Pose(self.current_pos[0], self.current_pos[1], self.current_pos[2])

    def __repr__(self) -> str:
        return f"PathFollower(current_pos={self.current_pos}, speed_mult={self.speed_mult}, total_distance={self.total_distance}, direction={self.direction}, distance_traveled={self.distance_traveled})"
