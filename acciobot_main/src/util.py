#!/usr/bin/env python
import move_base_msgs.msg
class Order(object):
    def __init__(self, items):
        self.items = items

    def fulfill_order(self):
        print("Beginning order")
        for item in self.items:
            item.fulfill_item()
        print("Finishing order")

class Station(object):
    def __init__(self, location, station_id, navigator):
        # location is type posestamped
        self.location = location
        self.station_id = station_id
        self.navigator = navigator

    # Navigates Fetch to the location of this station
    def attract_fetch(self):
        print("Going to station", self.station_id)
        self.navigator.move_to_posestamped(self.location)

class Item(object):
    def __init__(self, item, station, program):
        self.item = item
        self.station = station
        self.program = program

    def fulfill_item(self):
        self.go_to_item()
        self.locate_item()
        self.grab_and_drop_item()

    def go_to_item(self):
        self.station.attract_fetch()

    def locate_item(self):
        pass

    def grab_and_drop_item(self):
        print('Attempting to grab and drop item:', self.item)
        self.program.execute(self.item.feducial_id)
