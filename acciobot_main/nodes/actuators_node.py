#!/usr/bin/env python

import copy
import pickle

import rospy

import std_msgs.msg
import geometry_msgs.msg

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

from acciobot_main.msg import Item

import fetch_api

# import map_annotator.msg
# from map_annotator.srv import SendFetch, SendFetchResponse

import acciobot_main.msg
from acciobot_main.srv import HandleOrder

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

def create_box_marker():
    box_marker = Marker()
    box_marker.type = Marker.ARROW
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.1
    box_marker.scale.z = 0.1
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0
    return box_marker

class ActuatorServer(object):
    def __init__(self):
        # self.goto_pub = rospy.Publisher('/move_base_simple/goal', geometry_msgs.msg.PoseStamped)
        self.order_pub = rospy.Publisher('/handle_order', acciobot_main.msg.Order, latch=True)


        # rospy.Subscriber('/amcl_pose', geometry_msgs.msg.PoseWithCovarianceStamped, self._handle_pose_callback)
	self.items = []
        self.names = []
        self.quantity = []
        # filename = rospy.get_param('~pose_file')
        # try:
        #   with open(filename, "r") as f:
        #       self.names = pickle.load(f)

        # except EOFError:
        #   self.names = {}

        # self.pose = None

        # self.server = InteractiveMarkerServer('simple_marker')

        # self.name_markers = {}
        # self.name_controls = {}
        # pose_message = map_annotator.msg.PoseNames()
        # pose_message.poses = list(self.names.keys())

        # self.pose_pub.publish(pose_message)

        print("initiated actuator nodes with", self.names)
    # def _create_marker(self, name):
    #   self.name_markers[name] = InteractiveMarker()
    #   self.name_markers[name].header.frame_id = "map"
    #   self.name_markers[name].name = name
    #   self.name_markers[name].description = name

    #   self.name_markers[name].pose.position.x = self.pose.pose.pose.position.x
    #   self.name_markers[name].pose.position.y = self.pose.pose.pose.position.y

    #   # TODO(emersonn): REMEMBER THAT THIS IS 1 IF IT IS CRAZY LOL
    #   self.name_markers[name].pose.position.z = 1

    #   box_marker = create_box_marker()

    #   self.name_controls[name] = InteractiveMarkerControl()
    #   self.name_controls[name].interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    #   self.name_controls[name].always_visible = True

    #   self.name_controls[name].orientation.w = 1
    #   self.name_controls[name].orientation.x = 0
    #   self.name_controls[name].orientation.y = 1
    #   self.name_controls[name].orientation.z = 0

    #   new_controller_lol = InteractiveMarkerControl()
    #   new_controller_lol.orientation.w = 1
    #   new_controller_lol.orientation.x = 0
    #   new_controller_lol.orientation.y = 1
    #   new_controller_lol.orientation.z = 0
    #   new_controller_lol.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    #   new_controller_lol.orientation_mode = InteractiveMarkerControl.FIXED
    #   self.name_markers[name].controls.append(new_controller_lol)


    #   # self.name_controls[name].orientation.w = self.pose.pose.pose.orientation.w
    #   # self.name_controls[name].orientation.x = self.pose.pose.pose.orientation.x
    #   # self.name_controls[name].orientation.y = self.pose.pose.pose.orientation.y
    #   # self.name_controls[name].orientation.z = self.pose.pose.pose.orientation.z

    #   self.name_controls[name].markers.append(box_marker)
    #   self.name_markers[name].controls.append(self.name_controls[name])

    #   self.server.insert(self.name_markers[name], self._callback)
    #   self.server.applyChanges()

    # def _callback(self, dog):
    #   name = dog.marker_name
    #   pose = dog.pose

    #   rospy.logerr(str(self.names[name]))

    #   self.names[name].pose.pose.position.x = pose.position.x
    #   self.names[name].pose.pose.position.y = pose.position.y

    #   self.names[name].pose.pose.orientation = pose.orientation

    #   rospy.logerr(str(self.names[name]))

    # def _delete_marker(self, name):
    #   self.server.erase(name)
    #   self.server.applyChanges()

    def handle_send_fetch(self, request):
        rospy.loginfo("Attempting: " + str(request))

        #possible commands: create, cancel, status
        if request.command == request.ORDER:
            self._handle_create(request.items, request.quantity)
            # self._create_marker(request.name)
        # elif request.command == request.CANCEL:
        #   self._handle_delete(request.name)
        #   self._delete_marker(request.name)

        # elif request.command == request.GOTO:
        #   self._handle_goto(request.name)
        # elif request.command == request.LIST:
        #   rospy.loginfo('listing')
        else:
            rospy.logerr('none of these work')

        #send order message and receive response

        order_message = acciobot_main.msg.Order() #map_annotator.msg.PoseNames()
        order_message.items = self.items
	#order_message.names = self.names
        order_message.quantity = self.quantity

        self.order_pub.publish(order_message)

        # fetch_response = SendFetchResponse()
        # fetch_response.names = list(self.names.keys())
        # return fetch_response

    def _handle_create(self, items, quantity):
        #rospy.loginfo(str(self.pose))
        self.items = items
	self.quantity = quantity

    # def _handle_delete(self, name):
    #   if name not in self.names:
    #       rospy.loginfo('name not in names')
    #   else:
    #       del self.names[name]

    # def _handle_goto(self, name):
    #   new_goto = geometry_msgs.msg.PoseStamped()

    #   #rospy.loginfo(str(new_goto))
    #   rospy.loginfo("going to %s", name)
    #   rospy.loginfo(self.names)
    #   #rospy.loginfo(self.names[name])
    #   new_goto.header = self.names[name].header
    #   new_goto.pose = self.names[name].pose.pose
    #   #rospy.logerr("wtf" + str(self.names[name].pose.pose))

    #   #rospy.logerr("bro" + str(new_goto))
    #   self.goto_pub.publish(new_goto)

    def _handle_pose_callback(self, data):
        #rospy.loginfo("data being copied from amcl: " + str(data))
        self.pose = copy.deepcopy(data)

    def pickle_it_up_bro(self):
        with open("/home/team3/catkin_ws/src/cse481c/map_annotator/LOL.pickle", "w") as f:
            pickle.dump(self.names, f)

    def testerrr(self):
        order_message = acciobot_main.msg.Order() #map_annotator.msg.PoseNames()
        i = Item()
	i.item_name = "hi"
	i.item_id = 5
	order_message.items = [i]
	#order_message.names = ["self.names"]
        order_message.quantity = [5]

        self.order_pub.publish(order_message)


def main():
    rospy.init_node('acciobot_main_actuators')
    wait_for_time()

    server = ActuatorServer()
    move_service = rospy.Service('acciobot_main/handle_order', HandleOrder, server.handle_send_fetch)
    #server.testerrr()
    rospy.on_shutdown(server.pickle_it_up_bro)

    rospy.spin()

if __name__ == "__main__":
    main()

