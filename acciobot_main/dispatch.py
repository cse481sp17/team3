#!/usr/bin/env python

import rospy
from acciobot_main.srv import HandleOrder, HandleOrderRequest
from acciobot_main.msg import Item
import std_msgs.msg
import acciobot_main.msg

orders = {}

def printOrder(order):
	print "You have a new order!"
	orders[order.order_id] = order

def main():
	rospy.init_node('command_line_dispatch')
	rospy.Subscriber('handle_order', acciobot_main.msg.Order, printOrder)
	while True:
		raw_answer = raw_input("Type an id of a pending job to dispatch Fetch: ")
		if int(raw_answer) in orders.keys():
			print "Dispatch Fetch to fulfill:",orders[int(raw_answer)]
			# publish to backend
		


if __name__ == "__main__":
	main()
