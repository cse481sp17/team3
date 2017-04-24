#!/usr/bin/env python

import copy

import rospy

import geometry_msgs.msg

from map_annotator.srv import SendFetch, SendFetchResponse

def wait_for_time():
	while rospy.Time().now().to_sec() == 0:
		pass

class ActuatorServer(object):
	def __init__(self):
		self.goto_pub = rospy.Publisher('/move_base_simple/goal', geometry_msgs.msg.PoseStamped)	
		rospy.Subscriber('/amcl_pose', geometry_msgs.msg.PoseWithCovarianceStamped, self._handle_pose_callback)

		self.names = {}
		self.pose = None

	def handle_send_fetch(self, request):
		rospy.logerr("yo we got fam" + str(request))
		if request.command == request.CREATE:
			self._handle_create(request.name)
		elif request.command == request.DELETE:
			self._handle_delete(request.name)
		elif request.command == request.GOTO:
			self._handle_goto(request.name)
		elif request.command == request.LIST:
			rospy.loginfo('hey Karen')		

		fetch_response = SendFetchResponse()
		fetch_response.names = list(self.names.keys())
		return fetch_response

	def _handle_create(self, name):
		rospy.loginfo(str(self.pose))
		self.names[name] = self.pose

	def _handle_delete(self, name):
		if name not in self.names:
			rospy.loginfo('wtf bro')
		else:
			del self.names[name]

	def _handle_goto(self, name):
		new_goto = geometry_msgs.msg.PoseStamped()
		
		rospy.loginfo(str(new_goto))
		new_goto.header = self.pose.header		
		new_goto.pose = self.pose.pose.pose
		rospy.logerr("wtf" + str(self.pose.pose.pose))

		rospy.logerr("bro" + str(new_goto))
		self.goto_pub.publish(new_goto)

	def _handle_pose_callback(self, data):
		self.pose = copy.deepcopy(data)
		

def main():
	rospy.init_node('map_annotator_actuators')
	wait_for_time()

	server = ActuatorServer()

	move_service = rospy.Service('map_annotator/send_fetch', SendFetch, server.handle_send_fetch)
	
	rospy.spin()

if __name__ == "__main__":
	main()
