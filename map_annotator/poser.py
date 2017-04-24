#!/usr/bin/env python

import rospy
from map_annotator.srv import SendFetch, SendFetchRequest

def print_intro():
	print ("Welcome to the map annotator!")
	print ("Commands:")
	print ("   list: List saved poses.")
	print ("   save <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists.")
	print ("   delete <name>: Delete the pose given by <name>.")
	print ("   goto <name>: Sends the robot to the pose given by <name>.")
	print ("   help: Show this list of commands")

def get_info(send_fetch):
	answer = ""
	command = ""
	name = ""
	while answer != "quit":
		raw_answer = raw_input("> ")
		if raw_answer == "quit":
			quit()
		if raw_answer == "list":
			command = SendFetchRequest.LIST
		elif raw_answer == "help":
			print_intro()
		else:
			answer = raw_answer.partition(" ")[0]
			name = raw_answer.partition(" ")[2]
			if answer == "save":
				command = SendFetchRequest.CREATE
			elif answer == "delete":
				command = SendFetchRequest.DELETE
			elif answer == "goto":
				command = SendFetchRequest.GOTO
		try:
			response = send_fetch(command, name)
			print response
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))
		# response = pub.publish(fetcher)
		# print response
			
def call_list():
	# Work to list saved things
	print ("Poses:")



def main():
	print_intro()
	rospy.wait_for_service('map_annotator/send_fetch')
	send_fetch = rospy.ServiceProxy('map_annotator/send_fetch', SendFetch)
	get_info(send_fetch)


if __name__ == "__main__":
	main()	
