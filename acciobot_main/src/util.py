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

from geometry_msgs.msg import Pose, PoseStamped

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

soundhandle = SoundClient()
rospy.sleep(1)

voice = 'voice_kal_diphone'

clear_pub = rospy.Publisher('accio_clear_collisions', std_msgs.msg.Bool, queue_size=5)
coll_list_pub = rospy.Publisher('accio_collisions', acciobot_main.msg.CollisionList, queue_size=5)

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

def transform_to_pose(matrix):
    pose = Pose()
    pose.position.x = matrix[0, 3]
    pose.position.y = matrix[1, 3]
    pose.position.z = matrix[2, 3]
    x, y, z, w = tft.quaternion_from_matrix(matrix)
    pose.orientation.x = x
    pose.orientation.y = y
    pose.orientation.z = z
    pose.orientation.w = w
    return pose


def pose_to_transform(pose):
    q = pose.orientation
    matrix = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    matrix[0, 3] = pose.position.x
    matrix[1, 3] = pose.position.y
    matrix[2, 3] = pose.position.z
    return matrix

def transform_by_part(obj_pose_in_base, x, y, z):
    obj_mat_in_base = pose_to_transform(obj_pose_in_base)

    pregrasp_pose_in_obj = Pose()
    pregrasp_pose_in_obj.position.x = x
    pregrasp_pose_in_obj.position.y = y
    pregrasp_pose_in_obj.position.z = z

    pregrasp_pose_in_obj.orientation.w = 1

    pregrasp_mat_in_obj = pose_to_transform(pregrasp_pose_in_obj)

    pregrasp_mat_in_base = np.dot(obj_mat_in_base, pregrasp_mat_in_obj)
    pregrasp_pose = transform_to_pose(pregrasp_mat_in_base)

    ps1 = Pose()
    # ps1.header.frame_id = 'base_link'
    ps1 = copy.deepcopy(pregrasp_pose)

    return ps1

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
            done = False
            tries = 0

            while not done:
                #soundhandle.say('I am about to get an item, hurrah!', voice)
                done = item.fulfill_item()
                tries += 1

                if not done:
                    print("Couldn't find the item, going back to retry the entire item")
                    raw_input()
                    item.goback.execute()
                if tries == 2 and not done:
                    print('We tried two times and meh for item')
                    break

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
        print("Going to station", self.station_id, "press enter to continue")
        raw_input()
        self.navigator.move_to_posestamped(self.location) #TODO uncomment this

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
        self.torsoed = False

    def fulfill_item(self):
        self.go_to_item()
        desired_pose = self.locate_item()
        done = False

        tries = 0
        while not done:
            print("We're about to call into perceiving!")
            # raw_input()
            located_pose = self.locate_item()
            if located_pose == False:
                return False
            done = self.grab_and_drop_item(located_pose)

            tries += 1
            if not done:
                print('Retrying, we are on try', tries)
            if tries == 2 and not done:
                return done

        return done

    def go_to_item(self):
        self.station.attract_fetch()

    def locate_item(self, recursion_depth=None):
        # TODO(emersonn): UNCOMMENT THIS AS NECESSARY
        if recursion_depth is not None and recursion_depth > 5:
            return False

        new_msg = std_msgs.msg.Bool()
        new_msg.data = True
        clear_pub.publish(new_msg)

        # print('UNCOMMEN THIS')
        if not self.torsoed:
            self.shelf_program.execute()
            self.torsoed = True
        # TODO(emersonn): Read from arguments instead, this is HARDCODED!
        new_point_cloud = rospy.wait_for_message("cloud_in", PointCloud2)
        #new_point_cloud = rospy.wait_for_message("/head_camera/depth_registered/points", PointCloud2)

        self.seg_pipe_pub.publish(new_point_cloud)
        print('Looking on shelf:', self.shelf, 'for item', self.item.item_id)
        # print("Yo, we're sending a message to the segmentation pipeline")
        # print("Waiting for message LOL")

        # rospy.wait_for_message("accio_collisions", acciobot_main.msg.CollisionList)
        found_items = rospy.wait_for_message("accio_items", acciobot_main.msg.PerceivedItems)
        print("Got message, continuing...")

        #print("OUR MESSAGE IS AS FOLLOWS:", found_items)
        # print("MESSAGE")

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
        THRESHOLD = 0.03
        num_possible = 0
        for mark in found_items.tables[self.shelf].markers:
            second_biggest = max(mark.scale.x, mark.scale.y)
            print("Looking for", self.item.item_id, "with width", item_width, "and height", item_height)
            print('Possible marker', mark.scale, "heightdifference", abs(mark.scale.z - item_height), "widthdiff", abs(second_biggest - item_width))
            if abs(mark.scale.z - item_height) < THRESHOLD and abs(second_biggest - item_width) < THRESHOLD:
                num_possible = num_possible + 1

        if len(found_items.tables[self.shelf].markers) == 0 or num_possible == 0:
            if recursion_depth is None:
                recursion_depth = 0
            print('Found no items. Recursion depth is', recursion_depth)
            return self.locate_item(recursion_depth + 1)

        for mark in found_items.tables[self.shelf].markers:
            # print('Marker', mark)
            second_biggest = max(mark.scale.x, mark.scale.y)
            sqrd_wid = (second_biggest - item_width) ** 2
            sqrd_heit = (mark.scale.z - item_height) ** 2
            fork_spoons = math.sqrt(sqrd_wid + sqrd_heit)
            print('CURR SCALE:', mark.scale)

            if min_distance is None or fork_spoons < min_distance:
                min_distance = fork_spoons
                min_item = mark

        non_item_markers = []
        for shelf in range(4):
            for mark in found_items.tables[shelf].markers:
                if mark != min_item and mark.pose.position.x > min_item.pose.position.x - 0.10:
                    new_coll = acciobot_main.msg.BoxCollision()
                    width = max(mark.scale.x, mark.scale.y)
                    depth = min(mark.scale.x, mark.scale.y)

                    new_coll.pose = copy.deepcopy(mark.pose)
                    new_coll.scale = copy.deepcopy(mark.scale)

                    new_coll.scale.y = width
                    new_coll.scale.x = depth

                    non_item_markers.append(new_coll)
        print("We found", len(non_item_markers), "non-items")
        new_coll_list = acciobot_main.msg.CollisionList()
        new_coll_list.title = 'got_items'
        new_coll_list.collisions = copy.deepcopy(non_item_markers)

        coll_list_pub.publish(new_coll_list)

        # TODO if two items are the same pick the one that is closer
        # TODO once the bounding box was just really bad. also include left and right?

        print('FOUND MIN DISTANCE', min_distance, 'with desired pose', min_item.pose)
        print('SCALE:', min_item.scale)
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
            #print('Cheezit transform yo')
            # TODO(emersonn): TO FIX CHEEZITS WE WOULD NEED TO MOVE CLOSER SPECIFICALLY!!!!!
            desired_pose = transform_by_part(desired_pose, 0, 0, min_item.scale.z / 2 - 0.08)
        if self.item.item_id == 2 or self.item.item_id == 3:
            #print('Pasta transform')
            desired_pose = transform_by_part(desired_pose, 0, .02, min_item.scale.z / 2 + 0.02)

        # quaternion = (
        #     desired_pose.orientation.x,
        #     desired_pose.orientation.y,
        #     desired_pose.orientation.z,
        #     desired_pose.orientation.w)
        # euler = tft.euler_from_quaternion(quaternion)
        # roll = euler[0]
        # pitch = euler[1]
        # yaw = euler[2]
        """
        if yaw < 1 or yaw > 2:
            print("roll", roll, "pitch", pitch, "yaw", yaw)
            print('Bad yaw Try again!')
            return self.locate_item()

        """
        #print("good!", "roll", roll, "pitch", pitch, "yaw", yaw)
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
        # print('NEED TO UNCOMMENT THISSSSS util.py line 45')

        # TODO(emersonn): LOL! THIS IS STUFF
        # print("HEY IT'S ME. HANNAH. HANNAH BAKER. WE NEED TO COMMENT THIS OUT. FOR FIDUCIALS")
        # done = self.program.execute(self.item.feducial_id)
        # if done:
        #     self.drop.execute()
        # return done

        # print("Attempting to grab item at:", desired_pose) TODO(emersonn): We
        # should also move out before we move it
        print("DESIRED POSE", desired_pose)
        #self.goback.execute()
        done = self.program.execute(desired_pose=desired_pose)
        if done:
            # TODO karan make sure these only happen in order
            self.goback.execute()
            new_msg = std_msgs.msg.Bool()
            new_msg.data = True
            clear_pub.publish(new_msg)
            self.drop.execute()
            self.raisez.execute()
            self.tuck.execute()
            #self.goforward.execute()
        return done
