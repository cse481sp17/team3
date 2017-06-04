#!/usr/bin/env python
import rospy

from util import Order, Station, Item
import geometry_msgs.msg
import pickle
import actionlib
import move_base_msgs.msg
import std_msgs.msg
try:
    from queue import Queue
except ImportError:
    from Queue import Queue

from sensor_msgs.msg import PointCloud2

import acciobot_main.msg

class OrderHandler(object):
    def __init__(self, station_handler, program_handler, status_pub):
        self.order_queue = [] # LOL it's not a queue
        self.station_handler = station_handler
        self.program_handler = program_handler
        self.pub = status_pub

        self.seg_pipe_pub = rospy.Publisher('seg_pipe_go', PointCloud2, queue_size=10)

        # Until we figure out how to pull off orders, we can hardcode orders here
        # assumes STATIONS_FILE has a key called "section1" with the right location
        # TODO read from the web / command line
        # TODO(emersonn): We need to wrap every item with the item wrapper found in utils IF the items we are getting are of type Item Message
        available_stations = self.station_handler.get_station_ids()
        print(available_stations)
        available_stations.remove(0) # cashier is at 0
        # for num in available_stations:
        #     self.order_queue.append(self.make_fake_order_single_item(num))

    def big_fake_order(self):
        station_number = 0
        fake_station = self.station_handler.get_station(station_number)
    	first_item = acciobot_main.msg.Item
    	first_item.item_name = "Cheezits"
    	first_item.item_id = 1
    	first_item.item_type = "cheezitNEWESTNEW"
    	first_item.quantity = 4
    	first_item.feducial_id = 14
        first_program = self.program_handler.get_program("cheezitNEWESTNEW")
        first_drop = self.program_handler.get_drop()
        first = Item(first_item, fake_station, first_program, first_drop)

    	third_item = acciobot_main.msg.Item
    	third_item.item_name = "Hairdye"
    	third_item.item_id = 3
    	third_item.item_type = "cheezitNEWESTNEW"
    	third_item.quantity = 1
    	third_item.feducial_id = 0
        fake_program = self.program_handler.get_program("hairnewaction")
        third = Item(third_item, fake_station, fake_program, first_drop)
        fake_order = Order([third, first], self.pub)
        return fake_order

    def make_fake_order_single_item(self, station_number):
        fake_station = self.station_handler.get_station(station_number)
        fake_program = self.program_handler.get_program("box")
        drop_program = self.program_handler.get_drop()
        new_item_msg = acciobot_main.msg.Item()
        new_item_msg.item_name = str(station_number)
        new_item_msg.item_id = station_number
        new_item_msg.quantity = 420
        new_item_msg.item_type = "box"
        new_item_msg.feducial_id = 10

        fake_item = Item(new_item_msg, fake_station, fake_program, drop_program)

        fake_order = Order([fake_item], self.pub)
        return fake_order

    def remove_order(self, order_id):
        # TODO fix style
        for i, order in enumerate(self.order_queue):
            if order.order_msg.order_id == order_id:
                left_half = self.order_queue[0:i]
                right_half = self.order_queue[i + 1:len(self.order_queue)]
                self.order_queue = left_half + right_half
                return order

    # TODO karan check if this works / lock
    def add_order(self, order_msg):
        items = order_msg.items
        itemList = []
        for item in items:
            for i in range(item.quantity):
                program = self.program_handler.get_program(item.item_type)
                drop = self.program_handler.get_drop()
                tuck = self.program_handler.get_tuck()
                # TODO(emersonn): HARDCODED SHELF AND STATION. Need to put these correctly!
                curr_shelf = 2
                it = Item(
                    item,
                    self.station_handler.get_station(1), program, drop, tuck, self.seg_pipe_pub,
                    curr_shelf, self.program_handler.get_shelf(curr_shelf),
                    self.program_handler.get_raise(),
                    self.program_handler.get_forward(),
                    self.program_handler.get_backward()
                )
                itemList.append(it)
        order = Order(itemList, order_msg, self.pub)
        self.order_queue.append(order)

    # should be called before we put an order on the queue
    # sort items by station id
    def optimize_order(self):
        pass

    # TODO talk with karen and yen and subscribe to their order thing and update this callback
    def order_callback(self, order_msg):
        pass
        #self.order_queue.put(order)
