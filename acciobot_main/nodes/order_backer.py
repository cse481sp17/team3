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
from acciobot_main.srv import HandleOrder, HandleOrderResponse
import acciobot_main.msg
import fetch_api

iid = 1
def next_id():
    global iid
    res = iid
    iid += 1
    return res

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
        self.order_pub = rospy.Publisher('/handle_order', acciobot_main.msg.Order, latch=True)
        self.cancel_pub = rospy.Publisher('/cancel_order', std_msgs.msg.String)
	self.items = []
	self.cancel_id = 0

    def handle_send_fetch(self, request):
        rospy.loginfo("Attempting: " + str(request))

        #possible commands: create, cancel, status
        if request.command == request.ORDER:
            self.items = request.items
            self.order_id = next_id()
            rospy.loginfo("ORDER request")
        else:
            rospy.loginfo("CANCEL request")
            self.cancel_id = request.cancel_id
            self.cancel_pub.publish(std_msgs.msg.String(str(self.cancel_id)))
            response = HandleOrderResponse()
            response.id = 0
            response.response_string = "cancel"
            return response
            

        #send order message and receive response

        order_message = acciobot_main.msg.Order() 
        order_message.items = self.items
	order_message.order_id = self.order_id

        self.order_pub.publish(order_message)
	response = HandleOrderResponse()
	response.id = self.order_id
        response.response_string = "order" 
	return response

    def _handle_pose_callback(self, data):
        self.pose = copy.deepcopy(data)

    def pickle_it_up_bro(self):
        with open("/home/team3/catkin_ws/src/cse481c/map_annotator/LOL.pickle", "w") as f:
            pickle.dump(self.names, f)

def main():
    rospy.init_node('acciobot_main_actuators')
    wait_for_time()

    server = ActuatorServer()
    move_service = rospy.Service('acciobot_main/handle_order', HandleOrder, server.handle_send_fetch)
    rospy.on_shutdown(server.pickle_it_up_bro)

    rospy.spin()

if __name__ == "__main__":
    main()

