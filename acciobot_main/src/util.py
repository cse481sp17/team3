#!/usr/bin/env python
import rospy

import copy
import math

import tf.transformations as tft

import numpy as np

import move_base_msgs.msg

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import acciobot_main.msg
import std_msgs.msg

clear_pub = rospy.Publisher('accio_clear_collisions', std_msgs.msg.Bool, queue_size=5)

HARDCODED_LOL_HEIGHTS = {
1: 0.21,
2: 0.18,
3: 0.12,
4: 0.26
}

HARDCODED_LOL_WIDTH = {
1: 0.16,
2: 0.09,
3: 0.07,
4: 0.20
}

class Order(object):
    def __init__(self, items, order_msg, status_pub):
        self.items = items
        self.order_msg = order_msg
        self.status_pub = status_pub

    def fulfill_order(self):
        message = "Beginning order " + str(self.order_msg.order_id)
        self.status_pub.publish(std_msgs.msg.String(message))
        print(message)
        for item in self.items:
            done = item.fulfill_item()
            if not done:
                message = "Aborting order " + str(self.order_msg.order_id)
                self.status_pub.publish(std_msgs.msg.String(message))
                print(message)
                return done
        message = "Finishing order " + str(self.order_msg.order_id)
        self.status_pub.publish(std_msgs.msg.String(message))
        print(message)
        return True

class Station(object):
    def __init__(self, location, station_id, navigator, station_obj):
        # location is type posestamped
        self.location = location
        self.station_id = station_id
        self.navigator = navigator
        self.station_obj = station_obj

    # Navigates Fetch to the location of this station
    def attract_fetch(self):
        print("Going to station WE COMMENTED THIS OUT", self.station_id)
        #self.navigator.move_to_posestamped(self.location) #TODO uncomment this

class Item(object):
    def __init__(self, item, station, program, drop, tuck, seg_pipe_pub, shelf, shelf_program, raisez, goforward, goback):
        self.item = item
        self.station = station
        self.program = program
        self.drop = drop
        self.tuck = tuck
        self.shelf = shelf
        self.raisez = raisez

        self.shelf_program = shelf_program

        self.goforward = goforward
        self.goback = goback

        self.seg_pipe_pub = seg_pipe_pub

    def fulfill_item(self):
        self.go_to_item()
        desired_pose = self.locate_item()
        done = self.grab_and_drop_item(desired_pose)
        return done

    def go_to_item(self):
        self.station.attract_fetch()

    def locate_item(self):
        self.shelf_program.execute()
        # TODO(emersonn): Read from arguments instead, this is HARDCODED!
        new_point_cloud = rospy.wait_for_message("cloud_in", PointCloud2)
        #new_point_cloud = rospy.wait_for_message("/head_camera/depth_registered/points", PointCloud2)

        self.seg_pipe_pub.publish(new_point_cloud)
        print('Looking on shelf:', self.shelf, 'for item')
        print("Yo, we're sending a message to the segmentation pipeline")
        print("Waiting for message LOL")

        # rospy.wait_for_message("accio_collisions", acciobot_main.msg.CollisionList)
        found_items = rospy.wait_for_message("accio_items", acciobot_main.msg.PerceivedItems)
        print("Got message, continuing...")

        # TODO(emersonn): THIS IS HARDCODED, WE NEED TO ACTUALLY FIND THE ITEM BY TSORING IT IN THE INFO BY ITS SHELF
        # TODO(emersonn): MAKE SURE THE STATION ACTUALLY HAS THE ITEMS LIST FOR SHELVES
        # // TODO(emersonn): We're assuming the desired_pose has an actual item
        # TODO(emersonn): If item type is left, we move this to the left, otherwise it is in the center
        desired_pose = None

        min_distance = None
        min_item = None
        item_width = HARDCODED_LOL_WIDTH[self.item.item_id]
        item_height = HARDCODED_LOL_HEIGHTS[self.item.item_id]
        # if len(found_items.tables[self.shelf].markers) > 0:
        #     desired_pose = found_items.tables[self.shelf].markers[0].pose
        for mark in found_items.tables[self.shelf].markers:
            # print('Marker', mark)
            sqrd_wid = (mark.scale.y - item_width) ** 2
            sqrd_heit = (mark.scale.z - item_height) ** 2
            fork_spoons = math.sqrt(sqrd_wid + sqrd_heit)

            if min_distance is None or fork_spoons < min_distance:
                min_distance = fork_spoons
                min_item = mark


        print('FOUND MIN DISTANCE', min_distance, 'with desired pose', min_item.pose)
        desired_pose = min_item.pose

        if self.item.item_id == 3:
            # desired_pose = self.transform_from_marker(desired_pose, 0, 0, min_item.scale.z / 2 + 0.02)
            pass

            # desired_pose.position.z += 0.05
        if self.item.item_id == 1:
            # desired_pose.position.y = desired_pose.position.y + min_item.scale.y / 2
            # desired_pose.position.z = min_item.scale.z / 2 + desired_pose.position.z - 0.08

            # desired_pose = self.transform_from_marker(desired_pose, - min_item.scale.x / 2 + 0.03, 0, min_item.scale.z / 2 - 0.03)

            # desired_pose = self.transform_from_marker(desired_pose, - min_item.scale.x / 2, 0, min_item.scale.z / 2 - 0.03)
            pass

        if desired_pose is None:
            # TODO ITEM NOT FOUND. Tell store worker.
            pass

        #desired_pose.position.z += 0.0
        return desired_pose

        # TODO(emersonn): For later: make message of segmented items, and wait for message

    def transform_from_marker(self, ps, x, y, z):
        p = copy.deepcopy(ps)
        ps1 = copy.deepcopy(ps)
        print(p)
        orientation_arr = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        quat_matrix = tft.quaternion_matrix(orientation_arr)
        print(quat_matrix)
        quat_matrix[0][3] = p.position.x
        quat_matrix[1][3] = p.position.y
        quat_matrix[2][3] = p.position.z

        print(quat_matrix)
        objTpg = np.matrix('1 0 0 ' + str(x) + '; 0 1 0 ' + str(y) + '; 0 0 1 ' + str(z) + '; 0 0 0 1')
        blTpg = np.dot(quat_matrix, objTpg)
        print(blTpg)

        print(blTpg.shape)
        ps1.position.x = blTpg[0, 3]
        ps1.position.y = blTpg[1, 3]
        ps1.position.z = blTpg[2, 3]

        return ps1

    def grab_and_drop_item(self, desired_pose=None):
        print('Attempting to grab and drop item:', self.item, self.item.feducial_id)
        print('NEED TO UNCOMMENT THISSSSS util.py line 45')

        # TODO(emersonn): LOL! THIS IS STUFF
        print("HEY IT'S ME. HANNAH. HANNAH BAKER. WE NEED TO COMMENT THIS OUT. FOR FIDUCIALS")
        # done = self.program.execute(self.item.feducial_id)
        # if done:
        #     self.drop.execute()
        # return done

        # print("Attempting to grab item at:", desired_pose)
        # TODO(emersonn): We should also move out before we move it
        print("WTF DESIRED POSE????", desired_pose)
        done = self.program.execute(desired_pose=desired_pose)
        if done:
            self.goback.execute()
            new_msg = std_msgs.msg.Bool()
            new_msg.data = True
            clear_pub.publish(new_msg)
            self.drop.execute()
            self.raisez.execute()
            self.tuck.execute()
            self.goforward.execute()

        return done
