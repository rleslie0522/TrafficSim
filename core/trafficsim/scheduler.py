#!/usr/bin/env python3


# ========================================================================================
#
# NAME:         prs_worldinit.py
# DESCRIPTION:  Provides services to assist with automated train scheduling and movement.
# AUTHORS:      CALVIN EARNSHAW, FAVOUR JAM, KACPER KOMNATA, REBEKAH LESLIE, ANDREAS MAITA
#
# ========================================================================================


# ========================================================================================
#
# IMPORT DEPENDENCIES
#
# ========================================================================================

from datetime import datetime
import requests
import pathlib
import csv

import rclpy
from rclpy.node import Node
from pyrobosim_msgs.srv import RequestWorldState
from trafficsim_interfaces.srv import CRSDepartureLookup, APIRailServiceLookup, CRSAllDepartures


# ========================================================================================
#
# GLOBAL DEFINITIONS
#
# ========================================================================================

# Authentication Details for https://api.rtt.io/ - for non-commercial use only.
RTT_API_AUTH = ("rttapi_calvin.earnshaw", "d382dbe3efc6eb806a6a836e74f11d6052558c50")

# Station Dataset CSV - used for retrieving CRS codes for Railway Stations.
stations_csv = csv.reader(open(pathlib.Path(__file__).parent.parent.parent.parent.parent.parent.resolve().joinpath("../src/trafficsim/station_dataset/stations.csv"), "r"), delimiter=",")
STATIONS_CRS = {}

# Format station names to match the names used in Pyrobosim.
for station in stations_csv:
    crs = station[3]
    name = station[0].replace("&", "and").rstrip().replace(" ", "_")
    STATIONS_CRS[name] = crs


# ========================================================================================
#
# CLASS DEFINITIONS
#
# ========================================================================================


# ----------------------------------------------------------------------------------------
#
# NAME:         RailTrafficScheduler
# DESCRIPTION:  A node containing services which connect to the RTT.io API.
# PARAMETERS:   none
#
# ----------------------------------------------------------------------------------------

class RailTrafficScheduler(Node):

    # ------------------------------------------------------------------------------------
    #
    # Class Constructor
    #
    # ------------------------------------------------------------------------------------
    def __init__(self):
        super().__init__("RailTrafficScheduler")

        # Create PyRoboSim Client to access current world state.
        self.world_state_service = self.create_client(
            RequestWorldState,
            'request_world_state'
        )

        # Create services for next departure, service details, and all departures.
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

        # Used when calling the /request_next_departure service from the Train nodes to
        # keep track of which services have been run by a previous train.
        self._departure_index = {}
    
    # ------------------------------------------------------------------------------------
    #
    # NAME:         departures_callback
    # DESCRIPTION:  A callback function which returns the next departure from a given
    #               station passed in by service call.
    # PARAMETERS:   See trafficsim_interfaces/srv/CRSDepartureLookup.srv
    #               - request: A CRSDepartureLookup request, passed in via ROS2.
    # RETURNS:      - response: A CRSDepartureLookup response, passed out via ROS2.
    #
    # ------------------------------------------------------------------------------------
    def departures_callback(self, request, response):
        # Get the 3-character CRS code for the station name passed in through the request.
        crs = STATIONS_CRS[request.origin]

        url = f"https://api.rtt.io/api/v1/json/search/{crs}/{datetime.today().strftime("%Y")}/{datetime.today().strftime("%m")}/{datetime.today().strftime("%d")}"

        if url not in self._departure_index.keys():
            self._departure_index[url] = 0

        # Call the RTT.io API and return the station schedule for the given station for today.
        api_response = requests.get(url, auth=RTT_API_AUTH)

        # Get the status code and JSON results of the API call.
        api_response_status = api_response.status_code
        api_response = api_response.json()

        # If the API returns an OK (200) code, then retrieve details of the next departure
        # from this station. Otherwise, return an error with error code presented in ROS 2 server log.
        if api_response_status == 200:

            # Return status code 999 (used within other nodes to signal no more movements) if the departure index for
            # that particular station is the same as the total number of services originating from that station.
            if self._departure_index[url] == len(api_response["services"]) - 1 and request.lookup_only == False:
                self.get_logger().warn(f"No more departures from {crs}.")
                response.api_status_code = str(999)
                return response
            
            # Set response object to values from API.
            # A description of values is available here: https://www.realtimetrains.co.uk/about/developer/pull/docs/locationlist/
            response.api_status_code = str(api_response_status)

            response.service_uid = str(api_response["services"][self._departure_index[url]]["serviceUid"])
            response.destination = str(api_response["services"][self._departure_index[url]]["locationDetail"]["destination"][0]["description"])
            response.origin_start_time = str(api_response["services"][self._departure_index[url]]["locationDetail"]["origin"][0]["publicTime"])
            response.destination_arrival_time = str(api_response["services"][self._departure_index[url]]["locationDetail"]["destination"][0]["publicTime"])
            response.headcode = str(api_response["services"][self._departure_index[url]]["trainIdentity"])
            response.atoc_name = str(api_response["services"][self._departure_index[url]]["atocName"])

            # Increment departure index for this url if not a lookup only. This is to allow the next train requesting departure
            # to take a different service, not the service that has just been collected by another train.
            if request.lookup_only == False:
                self._departure_index[url] += 1

            # Log successful service outcome.
            self.get_logger().info(f"Retrieved departure information for {crs}")

            return response
        else:
            response.api_status_code = str(api_response_status)
            self.get_logger().error(f"Cannot retrieve departure information for {crs} - RTT API returned error code {str(api_response_status)}")
            return response
    
    # ------------------------------------------------------------------------------------
    #
    # NAME:         all_departures_callback
    # DESCRIPTION:  A callback function which returns all Service UIDs originating from a
    #               particular station today.
    # PARAMETERS:   See trafficsim_interfaces/srv/CRSAllDepartures.srv
    #               - request: A CRSAllDepartures request, passed in via ROS2.
    # RETURNS:      - response: A CRSAllDepartures response, passed out via ROS2.
    #
    # ------------------------------------------------------------------------------------
    def all_departures_callback(self, request, response):
        # Collect the request parameters - we retrieve the 3-digit code for a station here.
        crs = STATIONS_CRS[request.origin]
        year = str(request.year)
        month = str(request.month)
        date = str(request.date)

        # Call the RTT.io API and return the station schedule for the given station for today.
        api_response = requests.get(f"https://api.rtt.io/api/v1/json/search/{crs}/{year}/{month}/{date}", auth=RTT_API_AUTH)

        # Get the status code and JSON results of the API call.
        api_response_status = api_response.status_code
        api_response = api_response.json()

        # If the API returns an OK (200) code, then retrieve details of the next departure
        # from this station. Otherwise, return an error with error code presented in ROS 2 server log.
        if api_response_status == 200:
            # Create Response object to return in the callback.
            response.api_status_code = str(api_response_status)

            # Get all Service UIDs from the api_response json result that START AT the station passed in, and are of TRAIN type.
            # A description of values is available here: https://www.realtimetrains.co.uk/about/developer/pull/docs/locationlist/
            service_list = [
                service["serviceUid"] for service in api_response["services"]
                if service["locationDetail"]["displayAs"] == "ORIGIN"
                and service["serviceType"] == "train"
            ]

            response.service_uid_list = service_list

            return response
        else:
            response.api_status_code = str(api_response_status)
            self.get_logger().error(f"Cannot retrieve departure information for {crs} - RTT API returned error code {str(api_response_status)}")
            return response
    
    # ------------------------------------------------------------------------------------
    #
    # NAME:         service_details_callback
    # DESCRIPTION:  A callback function which returns all intermediate stops between an
    #               origin and destination station.
    # PARAMETERS:   See trafficsim_interfaces/srv/APIRailServiceLookup.srv
    #               - request: A CRSAllDepartures request, passed in via ROS2.
    # RETURNS:      - response: A CRSAllDepartures response, passed out via ROS2.
    #
    # ------------------------------------------------------------------------------------
    def service_details_callback(self, request, response):
        # Get parameters from request argument to pass to RTT.io API.
        service_uid = str(request.service_uid)
        year = str(request.year)
        month = str(request.month)
        date = str(request.date)

        # Call the RTT.io API and return the train service given service_uid, year, month, and date.
        api_response = requests.get(f"https://api.rtt.io/api/v1/json/service/{service_uid}/{year}/{month}/{date}", auth=RTT_API_AUTH)

        # Get the status code and JSON results of the API call.
        api_response_status = api_response.status_code
        api_response = api_response.json()

        # If the API returns an OK (200) code, then retrieve details of the train service.
        # Otherwise, return an error with error code presented in ROS 2 server log.
        if api_response_status == 200:
            # Set response object to values from API.
            # A description of values is available here: https://www.realtimetrains.co.uk/about/developer/pull/docs/locationlist/
            response.api_status_code = str(api_response_status)

            response.origin = str(api_response["origin"][0]["description"])
            response.destination = str(api_response["destination"][0]["description"])

            # Generate list of stops.
            stops = []
            for station_call in api_response["locations"][1:]:
                stops.append(str(station_call["description"]))
            
            response.stops = stops

            return response
        else:
            response.api_status_code = api_response.status_code
            self.get_logger().error(f"Cannot retrieve departure information for {crs} - RTT API returned error code {api_response.status_code}")
            return response


# ----------------------------------------------------------------------------------------
#
# NAME:         main
# DESCRIPTION:  The main entry point for this module.
# PARAMETERS:   none
# RETURNS:      none
#
# REFERENCES:   - https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html#getting-a-result
#
# ----------------------------------------------------------------------------------------

def main():
    rclpy.init()
    rtc = RailTrafficScheduler()
    rclpy.spin(rtc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
