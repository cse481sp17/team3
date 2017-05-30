#!/usr/bin/env python
import rospy

import move_base_msgs.msg

from sensor_msgs.msg import PointCloud2

import acciobot_main.msg

class Order(object):
    def __init__(self, items, order_msg):
        self.items = items
        self.order_msg = order_msg

    def fulfill_order(self):
        print("Beginning order")
        for item in self.items:
            done = item.fulfill_item()
            if not done:
                print("breaking early")
                return done
        print("Finishing order")

class Station(object):
    def __init__(self, location, station_id, navigator):
        # location is type posestamped
        self.location = location
        self.station_id = station_id
        self.navigator = navigator

    # Navigates Fetch to the location of this station
    def attract_fetch(self):
        print("Going to station WE COMMENTED THIS OUT", self.station_id)
        #self.navigator.move_to_posestamped(self.location) TODO uncomment this

class Item(object):
    def __init__(self, item, station, program, drop, seg_pipe_pub):
        self.item = item
        self.station = station
        self.program = program
        self.drop = drop

        self.seg_pipe_pub = seg_pipe_pub

    def fulfill_item(self):
        self.go_to_item()
        self.locate_item()
        done = self.grab_and_drop_item()
        return done

    def go_to_item(self):
        self.station.attract_fetch()

    def locate_item(self):
        # TODO(emersonn): Read from arguments instead, this is HARDCODED!
        new_point_cloud = rospy.wait_for_message("cloud_in", PointCloud2)
        self.seg_pipe_pub.publish(new_point_cloud)
        print("Yo, we're sending a message to the segmentation pipeline")
        print("Waiting for message LOL")

        rospy.wait_for_message("accio_collisions", acciobot_main.msg.CollisionList)
        print("Got message, continuing...")

        # TODO(emersonn): For later: make message of segmented items, and wait for message

    def grab_and_drop_item(self):
        print('Attempting to grab and drop item:', self.item, self.item.feducial_id)
        print('NEED TO UNCOMMENT THISSSSS util.py line 45')
        done = self.program.execute(self.item.feducial_id)
        if done:
            self.drop.execute()
        return done
