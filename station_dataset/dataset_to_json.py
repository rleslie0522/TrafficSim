from bs4 import BeautifulSoup
import json

soup = BeautifulSoup(open('/home/calvinearnshaw/ros2_ws/src/trafficsim/station_dataset/Category_Railway_stations_served_by_ScotRail.xml', 'r'), 'xml')

stations_json = {}
lines_json = {}

lines = {
    "ECML": ["Aberdeen", "Portlethen", "Stonehaven", "Laurencekirk", "Montrose", "Arbroath", "Carnoustie", "Monifieth", "Dundee", "Leuchars", "Cupar", "Springfield", "Ladybank", "Markinch", "Kirkcaldy", "Kinghorn", "Burntisland", "Aberdour", "Dalgety_Bay", "Inverkeithing", "North_Queensferry", "Dalmeny", "Haymarket", "Edinburgh_Waverley"],
    "EDI_GLA": ["Edinburgh_Waverley", "Haymarket", "Edinburgh_Park", "Linlithgow", "Polmont", "Falkirk_High", "Croy", "Lenzie", "Bishopbriggs", "Glasgow_Queen_Street"]
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

with open('station_dataset/RailStationCoords.json', 'w') as f:
    json.dump(stations_json, f)

with open('station_dataset/RailLines.json', 'w') as f:
    json.dump(lines_json, f)
