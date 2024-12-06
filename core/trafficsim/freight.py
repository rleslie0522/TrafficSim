import time
from typing import Optional

from pyrobosim.core.objects import Object
from pyrobosim.core.world import World
from rclpy.node import Node
from trafficsim.pathing import Station, StationGraph
from trafficsim_interfaces.msg import SpawnFreight, ClaimFreight, DeliverFreight, ToggleFreightSpawning

class FreightObject(Object):
    def __init__(self, parent, id: int, destination: str, **kwargs):
        self.destination = destination
        super().__init__(category="apple", name=str(id), parent=parent, inflation_radius=0.0, **kwargs)

class GlobalFreightManager(Node):
    def __init__(self, max_freight: int, station_graph: StationGraph, ros_world: World, **kwargs):
        super().__init__("Freight_Manager", **kwargs)
        self.max_freight = max_freight
        self.station_graph = station_graph
        self.ros_world = ros_world
        self.freight_id = 0
        self.freight_objects = {}
        self.spawn_freight_enabled = False

        self.spawn_freight_timer = self.create_timer(1.0, self.spawn_freight_timer_callback)
        self.freight_manager_subscriber = self.create_subscription(ToggleFreightSpawning, "toggle_freight_spawning", self.toggle_freight_spawning_callback, 10)
        self.freight_delivered_subcriber = self.create_subscription(DeliverFreight, "freight_delivered", self.freight_delivered_callback, 10)
        self.freight_spawn_publisher = self.create_publisher(SpawnFreight, "freight_spawn", 10)

    def spawn_freight_timer_callback(self):
        if len(self.freight_objects) < self.max_freight and self.spawn_freight_enabled:
            self.spawn_freight()

    def toggle_freight_spawning_callback(self, msg: ToggleFreightSpawning):
        self.spawn_freight_enabled = msg.enabled

    def spawn_freight(self, station_name: Optional[str] = None):
        origin_station = self.station_graph.get_random_node() if station_name is None else self.station_graph.get_node(station_name)
        if origin_station is None:
            self.get_logger().error(f"Station {station_name} does not exist.")
            return
        target_station = self.station_graph.get_random_node()
        self.get_logger().info(f"Spawning freight at {origin_station.name}")
        target_entity = self.ros_world.get_entity_by_name(f"{origin_station.name}_loc")
        new_freight = FreightObject(target_entity.children[0], self.freight_id, target_station.name)
        self.freight_objects[self.freight_id] = new_freight
        self.ros_world.add_object(object=new_freight)

        self.freight_spawn_publisher.publish(SpawnFreight(id=self.freight_id, location=origin_station.name, destination=target_station.name))
        self.freight_id += 1

    def freight_delivered_callback(self, msg: DeliverFreight):
        self.get_logger().info(f"Freight {msg.id} delivered to {msg.destination}")
        if msg.id in self.freight_objects:
            self.get_logger().info(f"Freight {msg.id} successfully delivered to {msg.destination}")
            time.sleep(1)
            self.ros_world.remove_object(self.freight_objects[msg.id])
            del self.freight_objects[msg.id]


class LocalFreightManager:
    def __init__(self, train_id: int):
        self.requests = {}
        self.external_claims = {}
        self.attempted_claim = None
        self.claimed = None

    def add_request(self, request: SpawnFreight):
        self.requests[request.id] = request

    def remove_request(self, request_id: int):
        del self.requests[request_id]

    def get_request(self, request_id: int):
        return self.requests[request_id]

    def try_claim_request(self, request_id: int):
        self.attempted_claim = request_id

    def add_external_claim(self, claim_msg: ClaimFreight):
        if claim_msg.id not in self.external_claims:
            self.external_claims[claim_msg.id] = []
        self.external_claims[claim_msg.id].append(claim_msg)

    def confirm_internal_claim(self) -> Optional[SpawnFreight]:
        if self.attempted_claim is None:
            return None
        self.claimed = self.attempted_claim
        self.attempted_claim = None
        return self.requests.get(self.claimed)

    def confirm_external_claim(self, request_id: int):
        if request_id == self.attempted_claim:
            self.attempted_claim = None
        del self.external_claims[request_id]
        self.remove_request(request_id)

    def get_best_train_for_request(self, request_id: int) -> Optional[int]:
        if len(self.external_claims[request_id]) == 0:
            return None
        sorted_claims = sorted(self.external_claims[request_id], key=lambda claim: claim.distance)
        closest_trains = []
        for claim in sorted_claims:
            if claim.distance == sorted_claims[0].distance:
                closest_trains.append(claim.train_id)
            else:
                break
        sorted_claims = sorted(closest_trains)
        return sorted_claims[0]

    def get_best_request_for_train(self, station_graph: StationGraph, current_loc: Station) -> Optional[SpawnFreight]:
        best_request = None
        best_distance = float("inf")
        for request in self.requests.values():
            dest_station = station_graph.get_node(request.location)
            if dest_station is None:
                continue
            distance = station_graph.get_dist_between_nodes(current_loc, dest_station)
            if distance < best_distance:
                best_distance = distance
                best_request = request
        return best_request
