#!/usr/bin/env python
from util import Order, Station, Item
import geometry_msgs.msg
import pickle
import actionlib
import move_base_msgs.msg
try:
    from queue import Queue
except ImportError:
    from Queue import Queue

import acciobot_main.msg

class OrderHandler(object):
    def __init__(self, station_handler, program_handler):
        self.order_queue = [] # LOL it's not a queue
        self.station_handler = station_handler
        self.program_handler = program_handler

        # Until we figure out how to pull off orders, we can hardcode orders here
        # assumes STATIONS_FILE has a key called "section1" with the right location
        # TODO read from the web / command line
        # TODO(emersonn): We need to wrap every item with the item wrapper found in utils IF the items we are getting are of type Item Message
        available_stations = self.station_handler.get_station_ids()
        available_stations.remove(0) # cashier is at 0
        for num in available_stations:
            self.order_queue.append(self.make_fake_order_single_item(num))

    def make_fake_order_single_item(self, station_number):
        fake_station = self.station_handler.get_station(station_number)
        fake_program = self.program_handler.get_program("box")

        new_item_msg = acciobot_main.msg.Item
        new_item_msg.item_name = str(station_number)
        new_item_msg.item_id = station_number
        new_item_msg.quantity = 420
        new_item_msg.item_type = "box"
        new_item_msg.feducial_id = 10

        fake_item = Item(new_item_msg, fake_station, fake_program)

        fake_order = Order([fake_item])
        return fake_order

    def remove_order(self, order_id):
        for (i, order in enumerate(self.order_queue)):
            if order.order_id == order_id:
                left_half = self.order_queue[0:i]
                right_half = self.order_queue[i + 1:len(self.order_queue)]
                self.order_queue = left_half + right_half
                return order

    # should be called before we put an order on the queue
    # sort items by station id
    def optimize_order(self):
        pass

    # TODO talk with karen and yen and subscribe to their order thing and update this callback
    def order_callback(self, order_msg):
        pass
        #self.order_queue.put(order)
