# ========================================================================================
#
# NAME:         dataset_to_json.py
# DESCRIPTION:  Converts a set of stations and lines into json files for use within the
#               world definition for Pyrobosim.
# AUTHORS:      CALVIN EARNSHAW, FAVOUR JAM, KACPER KOMNATA, REBEKAH LESLIE, ANDREAS MAITA
#
# ========================================================================================


# ========================================================================================
#
# IMPORT DEPENDENCIES
#
# ========================================================================================

import json
import pathlib
import csv


# ========================================================================================
#
# MAIN SCRIPT BODY
#
# ========================================================================================

# Define location of this file relative to rest of system.
parent_path = pathlib.Path(__file__).parent.resolve()

# Load up Stations CSV - the reference for this file: https://github.com/davwheat/uk-railway-stations/blob/main/stations.csv
stations_csv = csv.reader(open(parent_path.joinpath("stations.csv"), "r"), delimiter=",")

# Define stations and lines JSON files - these will be written out to separate json files to use within prs_worldinit.py
stations_json = {}
lines_json = {}

# Define all lines connecting stations in the Pyrobosim environment, in key/value pairs. The key is the name of the line,
# and can be defined as appropriate by the user. The value is an array of station names (the names must match exactly the
# station named posted in 'stations.csv', with spaces replaced with underscores '_'). The order in which stations are listed
# denotes a railway line connection (e.g. for ECML = ["Aberdeen", "Portlethen"], this denotes a railway line between
# Aberdeen and Portlethen).
lines = {
    # EAST COAST MAINLINE AND SURROUNDING BRANCH LINES
    "ECML": ["Aberdeen", "Portlethen", "Stonehaven", "Laurencekirk", "Montrose", "Arbroath", "Carnoustie", "Golf_Street", "Barry_Links", "Monifieth", "Balmossie", "Broughty_Ferry", "Dundee", "Leuchars", "Cupar", "Springfield", "Ladybank", "Markinch", "Glenrothes_with_Thornton", "Kirkcaldy", "Kinghorn", "Burntisland", "Aberdour", "Dalgety_Bay", "Inverkeithing", "North_Queensferry", "Dalmeny", "Edinburgh_Gateway", "South_Gyle", "Haymarket", "Edinburgh_Waverley", "Brunstane", "Musselburgh", "Wallyford", "Prestonpans", "Longniddry", "Drem", "Dunbar", "Berwick-Upon-Tweed"],
    "EDI_GLQ": ["Edinburgh_Waverley", "Haymarket", "Edinburgh_Park", "Linlithgow", "Polmont", "Falkirk_High", "Croy", "Lenzie", "Bishopbriggs", "Glasgow_Queen_Street"],
    "ECML_FIFE": ["Inverkeithing", "Rosyth", "Dunfermline_City", "Dunfermline_Queen_Margaret", "Cowdenbeath", "Lochgelly", "Cardenden", "Glenrothes_with_Thornton", "Markinch"],
    "KDY_LEV": ["Kirkcaldy", "Cameron_Bridge", "Leven"],
    "EDI_PTH": ["Ladybank", "Perth"],
    # HIGHLAND LINES
    "PTH_INV": ["Perth", "Dunkeld_and_Birnam", "Pitlochry", "Blair_Atholl", "Dalwhinnie", "Newtonmore", "Kingussie", "Aviemore", "Carrbridge", "Inverness"],
    "ABD_INV": ["Aberdeen", "Dyce", "Kintore", "Inverurie", "Insch", "Huntly", "Keith", "Elgin", "Forres", "Nairn", "Inverness_Airport", "Inverness"],
    # FAR NORTH LINES
    "INV_WCK": ["Beauly", "Muir_of_Ord", "Conon_Bridge", "Dingwall", "Alness", "Invergordon", "Fearn", "Tain", "Ardgay", "Lairg", "Golspie", "Brora", "Helmsdale", "Forsinard", "Georgemas_Junction", "Wick"],
    "WCK_TRS": ["Wick", "Georgemas_Junction", "Thurso"],
    "INV_KYL": ["Inverness", "Beauly", "Muir_of_Ord", "Conon_Bridge", "Dingwall", "Garve", "Achanalt", "Achnasheen", "Achnashellach", "Strathcarron", "Stromeferry", "Plockton", "Duirinish", "Kyle_of_Lochalsh"],
    # CENTRAL SCOTLAND LINES
    "PTH_GLQ": ["Perth", "Gleneagles", "Dunblane", "Bridge_of_Allan", "Stirling", "Larbert", "Croy"],
    "EDI_HEL": ["Edinburgh_Waverley", "Haymarket", "Edinburgh_Park", "Uphall", "Livingston_North", "Bathgate", "Armadale", "Blackridge", "Caldercruix", "Drumgelloch", "Airdrie", "Coatdyke", "Coatbridge_Sunnyside", "Blairhill", "Easterhouse", "Garrowhill", "Shettleston", "Carntyne", "Bellgrove", "High_Street", "Glasgow_Queen_Street", "Charing_Cross_(Glasgow)", "Partick", "Hyndland", "Jordanhill", "Scotstounhill", "Garscadden", "Yoker", "Clydebank", "Dalmuir", "Kilpatrick", "Bowling", "Dumbarton_East", "Dumbarton_Central", "Dalreoch", "Cardross", "Craigendoran", "Helensburgh_Central"],
    "EDI_GLC": ["Edinburgh_Waverley", "Haymarket", "Slateford", "Kingsknowe", "Wester_Hailes", "Curriehill", "Kirknewton", "Livingston_South", "West_Calder", "Addiewell", "Breich", "Fauldhouse", "Shotts", "Hartwood", "Cleland", "Carfin", "Holytown", "Bellshill", "Uddingston", "Newton", "Cambuslang", "Glasgow_Central"],
    # BORDERS RAILWAY
    "EDI_TWD": ["Edinburgh_Waverley", "Brunstane", "Newcraighall", "Shawfair", "Eskbank", "Newtongrange", "Gorebridge", "Stow", "Galashiels", "Tweedbank"],
    # WEST HIGHLAND LINES (INCL. DUMBARTON AND SURROUNDING AREAS)
    "DUM_BAL": ["Dumbarton_Central", "Dalreoch", "Renton", "Alexandria", "Balloch_Central"],
    "WESTHIGHLAND_OBN": ["Glasgow_Queen_Street", "Charing_Cross_(Glasgow)", "Partick", "Hyndland", "Jordanhill", "Scotstounhill", "Garscadden", "Yoker", "Clydebank", "Dalmuir", "Kilpatrick", "Bowling", "Dumbarton_East", "Dumbarton_Central", "Dalreoch", "Helensburgh_Upper", "Garelochhead", "Arrochar_and_Tarbet", "Ardlui", "Tyndrum_Lower", "Dalmally", "Loch_Awe", "Taynuilt", "Connel_Ferry", "Oban"],
    "WESTHIGHLAND_FTW": ["Tyndrum_Lower", "Upper_Tyndrum", "Bridge_of_Orchy", "Rannoch", "Corrour", "Tulloch", "Roy_Bridge", "Spean_Bridge", "Fort_William", "Banavie", "Corpach", "Loch_Eil_Outward_Bound", "Locheilside", "Glenfinnan", "Lochailort", "Beasdale", "Arisaig", "Morar", "Mallaig"],
    # GLASGOW AND WEST OF SCOTLAND LINES
    "GLQ_MLG": ["Glasgow_Queen_Street", "Charing_Cross_(Glasgow)", "Partick", "Hyndland", "Anniesland", "Westerton", "Bearsden", "Hillfoot", "Milngavie"],
    "GLQ_GLC": ["Glasgow_Queen_Street", "Charing_Cross_(Glasgow)", "Partick", "Exhibition_Centre", "Anderston", "Glasgow_Central"],
    "GLC_GOU": ["Glasgow_Central", "Anderston", "Exhibition_Centre", "Cardonald", "Hillington_East", "Hillington_West", "Paisley_Gilmour_Street", "Paisley_St_James", "Bishopton", "Langbank", "Woodhall", "Port_Glasgow", "Bogston", "Cartsdyke", "Greenock_Central", "Greenock_West", "Fort_Matilda", "Gourock"],
    "GLC_WMY": ["Glasgow_Central", "Anderston", "Exhibition_Centre", "Cardonald", "Hillington_East", "Hillington_West", "Paisley_Gilmour_Street", "Paisley_St_James", "Whinhill", "Drumfrochar", "Branchton", "Inverkip", "Wemyss_Bay"],
    "GLC_AYR": ["Glasgow_Central", "Anderston", "Exhibition_Centre", "Cardonald", "Hillington_East", "Hillington_West", "Paisley_Gilmour_Street", "Johnstone", "Milliken_Park", "Howwood", "Lochwinnoch", "Glengarnock", "Dalry", "Kilwinning", "Irvine", "Barassie", "Troon", "Prestwick_International_Airport", "Prestwick_Town", "Newton-On-Ayr", "Ayr"],
    "GLC_ARD": ["Glasgow_Central", "Anderston", "Exhibition_Centre", "Cardonald", "Hillington_East", "Hillington_West", "Paisley_Gilmour_Street", "Johnstone", "Milliken_Park", "Howwood", "Lochwinnoch", "Glengarnock", "Dalry", "Kilwinning", "Stevenston", "Saltcoats", "Ardrossan_South_Beach", "Ardrossan_Town", "Ardrossan_Harbour"],
    "GLC_LRG": ["Glasgow_Central", "Anderston", "Exhibition_Centre", "Cardonald", "Hillington_East", "Hillington_West", "Paisley_Gilmour_Street", "Johnstone", "Milliken_Park", "Howwood", "Lochwinnoch", "Glengarnock", "Dalry", "Kilwinning", "Stevenston", "Saltcoats", "Ardrossan_South_Beach", "West_Kilbride", "Fairlie", "Largs"],
    "GLC_PSC": ["Glasgow_Central", "Dumbreck", "Corkerhill", "Mosspark", "Crookston", "Hawkhead", "Paisley_Canal"],
    "GLC_CAR": ["Glasgow_Central", "Crossmyloof", "Pollokshaws_West", "Kennishead", "Priesthill_and_Darnley", "Nitshill", "Barrhead", "Dunlop", "Stewarton", "Kilmaurs", "Kilmarnock", "Auchinleck", "New_Cumnock", "Kirkconnel", "Sanquhar", "Dumfries", "Annan", "Gretna_Green", "Carlisle"],
    "GLC_STR": ["Glasgow_Central", "Crossmyloof", "Pollokshaws_West", "Kennishead", "Priesthill_and_Darnley", "Nitshill", "Barrhead", "Dunlop", "Stewarton", "Kilmaurs", "Troon", "Prestwick_International_Airport", "Prestwick_Town", "Newton-On-Ayr", "Ayr", "Maybole", "Girvan", "Barrhill", "Stranraer"]
}

# Generate stations (rooms) json file by iterating through each value (station) in the lines list, checking its existence within the json,
# and adding it to the stations_json dictionary. The stations_json dictionary contains key/value pairs, where the key is the station
# name, and the value is a LONGITUDE/LATITUDE pair of coordinates.
for station in stations_csv:
    coords = (station[2], station[1])
    name = station[0].replace("&", "and").rstrip().replace(" ", "_")
    for lineset in lines.items():
        if name in lineset[1]:
            stations_json[name] = coords

# Generate lines (hallways) by creating key/value pairs of "room" connections. These are stored in the lines_json dictionary.
# Iterate through each line defined in the lines dictionary.
for line in lines.items():
    # Iterate through each station defined within the current line.
    for i in range(0, len(line[1])-1):
        # Define a pyrobosim hallway connection, a tuple of two stations x_1 and x_2. We also define the same connection backwards, to
        # account for users typing stations in reverse order, e.g. (HAYMARKET, EDINBURGH) = (EDINBURGH, HAYMARKET).
        station_connection = (str(line[1][i]), str(line[1][i+1]))
        station_connection_alt = (str(line[1][i+1]), str(line[1][i]))

        # We check for existence of either station_connection or station_connection_alt in the lines_json dictionary. If neither exists,
        # we add a new line into the lines_json dictionary. The key is formatted as the LINE_NAME_<STARTING_STATION>_<ENDING_STATION>,
        # and the value is the station_connection pair previously defined.
        if station_connection not in lines_json.values() and station_connection_alt not in lines_json.values():
            lines_json[f'{str(line[0])}_{str(line[1][i]).upper()}_{str(line[1][i+1]).upper()}'] = station_connection

# Write out the station coordinates and railway lines to their respective json files. These are used by the pyrobosim world definition file to
# create rooms and hallways.
with open(parent_path.joinpath('RailStationCoords.json'), 'w') as f:
    json.dump(stations_json, f)

with open(parent_path.joinpath('RailLines.json'), 'w') as f:
    json.dump(lines_json, f)
