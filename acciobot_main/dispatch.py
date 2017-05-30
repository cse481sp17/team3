#!/usr/bin/env python

import rospy
from acciobot_main.srv import HandleOrder, HandleOrderRequest
from acciobot_main.msg import Item
import std_msgs.msg
import acciobot_main.msg

orders ={}


def addOrder(order):
	orders[order.order_id] = order
	print "You have a new order! Press Enter to view."

def printOrder(dispatch_pub):
	print
	while True:
		print "Type 'list' to see pending jobs, 'quit' to quit"
		print "Type an id of a pending job to dispatch Fetch"
		raw_answer = raw_input("> ")
		try:
			answer = int(raw_answer)
		except ValueError:
			if raw_answer == "quit":
				quit()
			printPending()
			continue
		if answer in orders.keys():
			print "Dispatch Fetch to fulfill:",orders[answer]
			dispatch_pub.publish(orders[answer])
			orders.pop(answer)
			if not orders:
				print
				print "No orders are currently pending"

def printPending():
	print "Pending Order Ids:"
	for x in orders.keys():
		print x
	print

def main():
	rospy.init_node('command_line_dispatch')
	rospy.Subscriber('handle_order', acciobot_main.msg.Order, addOrder)
	dispatch_pub = rospy.Publisher('dispatch_order', acciobot_main.msg.Order, latch=True, queue_size=10)
	rospy.sleep(1)
	printOrder(dispatch_pub)
	rospy.spin()

if __name__ == "__main__":
	main()
