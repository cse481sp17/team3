#!/usr/bin/env python
import geometry_msgs.msg
from util import Station
import pickle
# Contains a file with pickled poses of the stations (currently in form station1, station2, ... cashier)
STATIONS_FILE = "/home/team3/catkin_ws/src/cse481c/map_annotator/stations.pickle"

# key for cashier in STATIONS_FILE
CASHIER = "cashier"
CASHIER_ID = 0 # not flexible, just here for readability

# if the station keys in the STATION_FILE are labeled station1, station2, etc
#   then the id is after the prefix
STATION_PREFIX = "station"

class StationInformation(object):
    def __init__(self, navigator):
        self.navigator = navigator
        try:
            with open(STATIONS_FILE, "r") as f:
                    # Holds raw info from map annotator about stations
                    self.station_info = pickle.load(f)
        except EOFError:
            self.station_info = {}

        # Any translation between the station_info file to
        #   station ID occurs here
        # Cashier should be ID 0, and we'll assume that other
        #   stations are the number at the end of their name "station1", etc
        self.stations = {}
        for name in self.station_info:
            station_id = int(name[len(STATION_PREFIX)]) if name.startswith(STATION_PREFIX) else CASHIER_ID
            location = geometry_msgs.msg.PoseStamped()
            location.header = self.station_info[name].header
            location.pose = self.station_info[name].pose.pose
            self.stations[station_id] = Station(location, station_id, self.navigator)
        print(self.stations)

    def get_station(self, station_number):
        return self.stations[station_number]

    def get_cashier(self):
        return self.get_station(CASHIER_ID)
    
    def get_raw_info_by_name(self, station_number):
        name = self.get_name_from_id(station_number)
        return self.station_info[name]

    def get_station_ids(self):
        return self.stations.keys()

    def get_name_from_id(self, station_number):
        return STATION_PREFIX + str(station_number)
