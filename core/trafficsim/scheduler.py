#!/usr/bin/env python3


# ========================================================================================
#
# NAME:         prs_worldinit.py
# DESCRIPTION:  Automatically triggers train movement based on live timetable.
# AUTHORS:      CALVIN EARNSHAW, FAVOUR JAM, KACPER KOMNATA, REBEKAH LESLIE, ANDREAS MAITA
#
# ========================================================================================


# ========================================================================================
#
# IMPORT DEPENDENCIES
#
# ========================================================================================

import rclpy
from rclpy.node import Node
import requests
import pathlib
import csv
from pyrobosim_msgs.srv import RequestWorldState


# Authentication Details for https://api.rtt.io/ - for non-commercial use only.
RTT_API_AUTH = ("rttapi_calvin.earnshaw", "d382dbe3efc6eb806a6a836e74f11d6052558c50")

# Station Dataset CSV - used for retrieving CRS codes for Railway Stations.
STATION_DATASET = csv.reader(open(pathlib.Path(__file__).parent.parent.parent.parent.parent.resolve().joinpath("src/trafficsim/station_dataset/stations.csv"), "r"), delimiter=",")


class RailTrafficController(Node):

    def __init__(self):
        super().__init__("RailTrafficScheduler")

        self.world_state_service = self.create_client(
            RequestWorldState,
            'request_world_state'
        )


def main():
    print('Hi from trafficsim.')

    response = requests.get("https://api.rtt.io/api/v1/json/search/ABD", auth=RTT_API_AUTH)

    data = response.json()

    print(f"Loaded {len(data["services"])} train services departing from {data["location"]["name"]}")




if __name__ == '__main__':
    main()