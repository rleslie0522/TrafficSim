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
from trafficsim_interfaces.srv import CRSDepartureLookup, APIRailServiceLookup, CRSAllDepartures

from datetime import datetime


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

        self.rail_service_lookup_service = self.create_service(
            APIRailServiceLookup,
            "RailTrafficScheduler/request_service_details",
            self.service_details_callback
        )

        self.all_departures_service = self.create_service(
            CRSAllDepartures,
            "RailTrafficScheduler/request_all_departures",
            self.all_departures_callback
        )

        self._departure_index = 0
    
    def departures_callback(self, request, response):
        crs = STATIONS_CRS[request.origin]

        api_response = requests.get(f"https://api.rtt.io/api/v1/json/search/{crs}/{datetime.today().strftime("%Y")}/{datetime.today().strftime("%m")}/{datetime.today().strftime("%d")}", auth=RTT_API_AUTH)

        api_response_status = api_response.status_code

        api_response = api_response.json()

        if api_response_status == 200:
            if self._departure_index == len(api_response["services"]) - 1 and request.lookup_only == False:
                self.get_logger().warn(f"No more departures from {crs}.")
                response.api_status_code = str(999)
                return response
            
            response.api_status_code = str(api_response_status)

            response.service_uid = str(api_response["services"][self._departure_index]["serviceUid"])
            response.destination = str(api_response["services"][self._departure_index]["locationDetail"]["destination"][0]["description"])
            response.origin_start_time = str(api_response["services"][self._departure_index]["locationDetail"]["origin"][0]["publicTime"])
            response.destination_arrival_time = str(api_response["services"][self._departure_index]["locationDetail"]["destination"][0]["publicTime"])
            response.headcode = str(api_response["services"][self._departure_index]["trainIdentity"])
            response.atoc_name = str(api_response["services"][self._departure_index]["atocName"])

            if request.lookup_only == False:
                self._departure_index += 1

            self.get_logger().info(f"Retrieved departure information for {crs}")

            return response
        else:
            response.api_status_code = str(api_response_status)
            self.get_logger().error(f"Cannot retrieve departure information for {crs} - RTT API returned error code {str(api_response_status)}")
            return response
    
    def all_departures_callback(self, request, response):
        crs = STATIONS_CRS[request.origin]
        year = str(request.year)
        month = str(request.month)
        date = str(request.date)

        api_response = requests.get(f"https://api.rtt.io/api/v1/json/search/{crs}/{year}/{month}/{date}", auth=RTT_API_AUTH)

        api_response_status = api_response.status_code

        api_response = api_response.json()

        if api_response_status == 200:
            response.api_status_code = str(api_response_status)

            service_list = [service["serviceUid"] for service in api_response["services"] if service["locationDetail"]["displayAs"] == "ORIGIN" and service["serviceType"] == "train"]

            response.service_uid_list = service_list

            return response
        else:
            response.api_status_code = str(api_response_status)
            self.get_logger().error(f"Cannot retrieve departure information for {crs} - RTT API returned error code {str(api_response_status)}")
            return response
    
    def service_details_callback(self, request, response):
        service_uid = str(request.service_uid)
        year = str(request.year)
        month = str(request.month)
        date = str(request.date)

        api_response = requests.get(f"https://api.rtt.io/api/v1/json/service/{service_uid}/{year}/{month}/{date}", auth=RTT_API_AUTH)

        api_response_status = api_response.status_code

        api_response = api_response.json()

        if api_response_status == 200:
            response.api_status_code = str(api_response_status)

            response.origin = str(api_response["origin"][0]["description"])
            response.destination = str(api_response["destination"][0]["description"])

            stops = []

            for station_call in api_response["locations"][1:]:
                stops.append(str(station_call["description"]))
            
            response.stops = stops

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