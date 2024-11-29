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
from trafficsim_interfaces.srv import CRSDepartureLookup


# Authentication Details for https://api.rtt.io/ - for non-commercial use only.
RTT_API_AUTH = ("rttapi_calvin.earnshaw", "d382dbe3efc6eb806a6a836e74f11d6052558c50")

# Station Dataset CSV - used for retrieving CRS codes for Railway Stations.
stations_csv = csv.reader(open(pathlib.Path(__file__).parent.parent.parent.parent.parent.parent.resolve().joinpath("../src/trafficsim/station_dataset/stations.csv"), "r"), delimiter=",")
STATIONS_CRS = {}

for station in stations_csv:
    crs = station[3]
    name = station[0].replace("&", "and").rstrip().replace(" ", "_")
    STATIONS_CRS[name] = crs


class RailTrafficController(Node):

    def __init__(self):
        super().__init__("RailTrafficScheduler")

        self.world_state_service = self.create_client(
            RequestWorldState,
            'request_world_state'
        )

        self.departure_service = self.create_service(
            CRSDepartureLookup,
            'RailTrafficScheduler/request_next_departure',
            self.departures_callback
        )
    
    def departures_callback(self, request, response):
        crs = STATIONS_CRS[request.origin]

        api_response = requests.get(f"https://api.rtt.io/api/v1/json/search/{crs}", auth=RTT_API_AUTH)

        api_response_status = api_response.status_code

        api_response = api_response.json()

        if api_response_status == 200:
            response.api_status_code = str(api_response_status)

            response.destination = str(api_response["services"][0]["locationDetail"]["destination"][0]["description"])
            response.origin_start_time = str(api_response["services"][0]["locationDetail"]["origin"][0]["publicTime"])
            response.destination_arrival_time = str(api_response["services"][0]["locationDetail"]["destination"][0]["publicTime"])
            response.headcode = str(api_response["services"][0]["trainIdentity"])
            response.atoc_name = str(api_response["services"][0]["atocName"])

            self.get_logger().info(f"Retrieved departure information for {crs}")

            return response
        else:
            response.api_status_code = api_response.status_code
            self.get_logger().error(f"Cannot retrieve departure information for {crs} - RTT API returned error code {api_response.status_code}")
            return response


def main():
    rclpy.init()
    rtc = RailTrafficController()
    rclpy.spin(rtc)
    rclpy.shutdown()


if __name__ == '__main__':
    main()