from bs4 import BeautifulSoup
import json
import pathlib

parent_path = pathlib.Path(__file__).parent.resolve()

soup = BeautifulSoup(open(parent_path.joinpath('Category_Railway_stations_served_by_ScotRail.xml'), 'r'), 'xml')

stations_json = {}
lines_json = {}

lines = {
    "ECML": ["Aberdeen", "Portlethen", "Stonehaven", "Laurencekirk", "Montrose", "Arbroath", "Carnoustie", "Monifieth", "Dundee", "Leuchars", "Cupar", "Springfield", "Ladybank", "Markinch", "Kirkcaldy", "Kinghorn", "Burntisland", "Aberdour", "Dalgety_Bay", "Inverkeithing", "North_Queensferry", "Dalmeny", "Haymarket", "Edinburgh_Waverley"],
    "EDI_GLA": ["Edinburgh_Waverley", "Haymarket", "Edinburgh_Park", "Linlithgow", "Polmont", "Falkirk_High", "Croy", "Lenzie", "Bishopbriggs", "Glasgow_Queen_Street"],
    "ABD_INV": ["Aberdeen", "Dyce", "Kintore", "Inverurie", "Insch", "Huntly", "Keith", "Elgin", "Forres", "Nairn", "Inverness_Airport", "Inverness"],
    "INV_WCK": ["Inverness", "Beauly", "Muir_of_Ord", "Conon_Bridge", "Dingwall", "Alness", "Invergordon", "Fearn", "Tain", "Ardgay", "Lairg", "Golspie", "Brora", "Helmsdale", "Forsinard", "Georgemas_Junction", "Wick"],
    "WCK_TRS": ["Wick", "Georgemas_Junction", "Thurso"],
    "INV_KYL": ["Inverness", "Beauly", "Muir_of_Ord", "Conon_Bridge", "Dingwall", "Garve", "Achanalt", "Achnasheen", "Achnashellach", "Strathcarron", "Stromeferry", "Plockton", "Duirinish", "Kyle_of_Lochalsh"],
    "EDI_INV": ["Ladybank", "Perth", "Pitlochry", "Blair_Atholl", "Dalwhinnie", "Newtonmore", "Kingussie", "Aviemore", "Carrbridge", "Inverness"],
    "DEE_CRO": ["Dundee", "Invergowrie", "Perth", "Gleneagles", "Dunblane", "Bridge_of_Allan", "Stirling", "Larbert", "Falkirk_High"],
    "GLA_OBN": ["Glasgow_Queen_Street", "Dumbarton_Central", "Helensburgh_Upper", "Garelochhead", "Arrochar_and_Tarbet", "Ardlui", "Tyndrum_Lower", "Dalmally", "Loch_Awe", "Taynuilt", "Connel_Ferry", "Oban"],
    "GLA_FTW": ["Tyndrum_Lower", "Bridge_of_Orchy", "Rannoch", "Corrour", "Tulloch", "Roy_Bridge", "Spean_Bridge", "Fort_William"],
    "FTW_MAL": ["Fort_William", "Banavie", "Corpach", "Loch_Eil_Outward_Bound", "Locheilside", "Glenfinnan", "Lochailort", "Beasdale", "Arisaig", "Morar", "Mallaig"]
}

# Generate Stations
for station in soup.findAll("wpt"):
    coords = (station['lon'], station['lat'])
    name = station.contents[1].string.replace('railway', '').replace('station', '').replace('(Scotland)', '').rstrip().replace(" ", "_")
    for lineset in lines.items():
        if name in lineset[1]:
            stations_json[name] = coords

# Generate Lines
for line in lines.items():
    for i in range(0, len(line[1])-1):
        station_connection = (str(line[1][i]), str(line[1][i+1]))
        station_connection_alt = (str(line[1][i+1]), str(line[1][i]))

        if station_connection not in lines_json.values() and station_connection_alt not in lines_json.values():
            lines_json[f'{str(line[0])}_{str(line[1][i]).upper()}_{str(line[1][i+1]).upper()}'] = (str(line[1][i]), str(line[1][i+1]))

with open(parent_path.joinpath('RailStationCoords.json'), 'w') as f:
    json.dump(stations_json, f)

with open(parent_path.joinpath('RailLines.json'), 'w') as f:
    json.dump(lines_json, f)
