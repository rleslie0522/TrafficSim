from bs4 import BeautifulSoup
import json
import pathlib

parent_path = pathlib.Path(__file__).parent.resolve()

soup = BeautifulSoup(open(parent_path.joinpath('Category_Railway_stations_served_by_ScotRail.xml'), 'r'), 'xml')

stations_json = {}
lines_json = {}

lines = {
    "ECML": ["Aberdeen", "Portlethen", "Stonehaven", "Laurencekirk", "Montrose", "Arbroath", "Carnoustie", "Monifieth", "Dundee", "Leuchars", "Cupar", "Springfield", "Ladybank", "Markinch", "Glenrothes_with_Thornton", "Kirkcaldy", "Kinghorn", "Burntisland", "Aberdour", "Dalgety_Bay", "Inverkeithing", "North_Queensferry", "Dalmeny", "Haymarket", "Edinburgh_Waverley"],
    "FIFECIR": ["Inverkeithing", "Rosyth", "Dunfermline_City", "Cowdenbeath", "Lochgelly", "Cardenden", "Glenrothes_with_Thornton"],
    "KDY-LVN": ["Kirkcaldy", "Glenrothes_with_Thornton", "Cameron_Bridge", "Leven"],
    "EDI_GLA": ["Edinburgh_Waverley", "Haymarket", "Edinburgh_Park", "Linlithgow", "Polmont", "Falkirk_High", "Croy", "Lenzie", "Bishopbriggs", "Glasgow_Queen_Street"],
    "ABD_INV": ["Aberdeen", "Dyce", "Kintore", "Inverurie", "Insch", "Huntly", "Keith", "Elgin", "Forres", "Nairn", "Inverness_Airport", "Inverness"],
    "INV_WCK": ["Inverness", "Beauly", "Muir_of_Ord", "Conon_Bridge", "Dingwall", "Alness", "Invergordon", "Fearn", "Tain", "Ardgay", "Lairg", "Golspie", "Brora", "Helmsdale", "Forsinard", "Georgemas_Junction", "Wick"],
    "WCK_TRS": ["Wick", "Georgemas_Junction", "Thurso"],
    "INV_KYL": ["Inverness", "Beauly", "Muir_of_Ord", "Conon_Bridge", "Dingwall", "Garve", "Achanalt", "Achnasheen", "Achnashellach", "Strathcarron", "Stromeferry", "Plockton", "Duirinish", "Kyle_of_Lochalsh"],
    "EDI_INV": ["Ladybank", "Perth", "Pitlochry", "Blair_Atholl", "Dalwhinnie", "Newtonmore", "Kingussie", "Aviemore", "Carrbridge", "Inverness"],
    "DEE_CRO": ["Dundee", "Invergowrie", "Perth", "Gleneagles", "Dunblane", "Bridge_of_Allan", "Stirling", "Larbert", "Falkirk_High"],
    "GLA_OBN": ["Glasgow_Queen_Street", "Dumbarton_Central", "Helensburgh_Upper", "Garelochhead", "Arrochar_and_Tarbet", "Ardlui", "Tyndrum_Lower", "Dalmally", "Loch_Awe", "Taynuilt", "Connel_Ferry", "Oban"],
    "GLA_FTW": ["Tyndrum_Lower", "Bridge_of_Orchy", "Rannoch", "Corrour", "Tulloch", "Roy_Bridge", "Spean_Bridge", "Fort_William"],
    "FTW_MAL": ["Fort_William", "Banavie", "Corpach", "Loch_Eil_Outward_Bound", "Locheilside", "Glenfinnan", "Lochailort", "Beasdale", "Arisaig", "Morar", "Mallaig"],
    "EDI-TWD": ["Edinburgh_Waverley", "Brunstane", "Shawfair", "Eskbank", "Newtongrange", "Gorebridge", "Stow", "Galashiels", "Tweedbank"],
    "EDI-NBW": ["Edinburgh_Waverley", "Brunstane", "Musselburgh", "Wallyford", "Prestonpans", "Longniddry", "Drem", "North_Berwick"],
    "DRE-DBR": ["Drem", "Dunbar"],
    "GLA_WMB": ["Glasgow_Queen_Street", "Cardonald", "Hillington_West", "Paisley_Gilmour_Street", "Bishopton", "Langbank", "Woodhall", "Port_Glasgow", "Whinhill", "Drumfrochar", "Branchton", "Inverkip", "Wemyss_Bay"],
    "GLA_AYR": ["Paisley_Gilmour_Street", "Johnstone", "Milliken_Park", "Howwood", "Lochwinnoch", "Glengarnock", "Dalry", "Kilwinning", "Irvine", "Barassie", "Troon", "Prestwick_Town", "Newton-on-Ayr", "Ayr"],
    "KWN_ARD": ["Kilwinning", "Stevenston", "Saltcoats", "Ardrossan_Harbour"],
    "KWN_LGS": ["Saltcoats", "West_Kilbride", "Fairlie", "Largs"],
    "GLA_SRR": ["Glasgow_Queen_Street", "Crossmyloof", "Kennishead", "Nitshill", "Barrhead", "Dunlop", "Stewarton", "Kilmaurs", "Barassie", "Troon", "Prestwick_Town", "Newton-on-Ayr", "Ayr", "Maybole", "Girvan", "Barrhill", "Stranraer"],
    "GLA_CAR": ["Kilmaurs", "Kilmarnock", "Auchinleck", "New_Cumnock", "Kirkconnel", "Sanquhar", "Dumfries", "Annan", "Gretna_Green", "Carlisle"],
    "EDI_GLC": ["Haymarket", "Slateford", "Wester_Hailes", "Curriehill", "Kirknewton", "Livingston_South", "West_Calder", "Addiewell", "Breich", "Fauldhouse", "Shotts", "Hartwood", "Cleland", "Carfin", "Bellshill", "Uddingston", "Newton", "Cambuslang", "Glasgow_Queen_Street"],
    "EDI_HEL": ["Haymarket", "Edinburgh_Park", "Curriehill", "Kirknewton", "Livingston_North", "Bathgate", "Armadale", "Blackridge", "Caldercruix", "Drumgelloch", "Airdrie", "Coatbridge_Sunnyside", "Easterhouse", "Glasgow_Queen_Street", "Dumbarton_Central", "Helensburgh_Upper"]
}


# Generate Stations
for station in soup.findAll("wpt"):
    coords = (station['lon'], station['lat'])
    name = station.contents[1].string.replace('railway', '').replace('station', '').replace('(Scotland)', '').replace("(Fife)", "").rstrip().replace(" ", "_")
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
