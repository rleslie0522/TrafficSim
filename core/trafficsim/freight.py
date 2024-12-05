from typing import Optional

from pyrobosim.core.objects import Object
from pyrobosim.core.world import World
from rclpy.node import Node
from trafficsim.pathing import StationGraph

class FreightObject(Object):
    def __init__(self, parent, id: int, **kwargs):
        super().__init__(category="apple", name=str(id), parent=parent, inflation_radius=0.0, **kwargs)

class GlobalFreightManager(Node):
    def __init__(self, max_freight: int, station_graph: StationGraph, ros_world: World, **kwargs):
        super().__init__("Freight_Manager", **kwargs)
        self.max_freight = max_freight
        self.station_graph = station_graph
        self.ros_world = ros_world
        self.freight_id = 0
        self.freight_objects = []

        self.spawn_freight_timer = self.create_timer(1.0, self.spawn_freight_timer_callback)

    def spawn_freight_timer_callback(self):
        if len(self.freight_objects) < self.max_freight:
            self.spawn_freight()

    def spawn_freight(self, station_name: Optional[str] = None):
        target_station = self.station_graph.get_random_node() if station_name is None else self.station_graph.get_node(station_name)
        if target_station is None:
            self.get_logger().error(f"Station {station_name} does not exist.")
            return
        target_station = self.station_graph.get_node("Aberdeen")
        self.get_logger().info(f"Spawning freight at {target_station.name}")
        target_entity = self.ros_world.get_entity_by_name(f"{target_station.name}_loc")
        new_freight = FreightObject(target_entity.children[0], self.freight_id)
        self.freight_id += 1
        self.ros_world.add_object(object=new_freight)
        self.freight_objects.append(new_freight)
