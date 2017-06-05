#!/usr/bin/env python

import rospy
from acciobot_main.srv import HandleOrder, HandleOrderRequest
from acciobot_main.msg import Item
import std_msgs.msg
import acciobot_main.msg

orders ={}
finished ={}
global active

def addOrder(order):
	orders[int(order.order_id)] = order
	print "You have a new order! Press Enter to view."

def printOrder(dispatch_pub, customer_pub):
	global active
	print
	while True:
		if not active:

			print "Type 'list' to see pending jobs, 'quit' to quit"
			print "Type an id of a pending job to dispatch Fetch"
			print "Type an id of a completed job to notify customer"
			raw_answer = raw_input("> ")
			try:
				answer = int(raw_answer)
			except ValueError:
				if raw_answer == "quit":
					quit()
				printPending()
				continue
			if answer in orders.keys():
				active = answer
				item_names = [item.item_name + " x" + str(item.quantity) for item in orders[answer].items]
				print "Dispatch Fetch to fulfill:",item_names #orders[answer]
				dispatch_pub.publish(orders[answer])
#				orders.pop(answer)
				if not orders:
					print
					print "No orders are currently pending"
				pass
			elif answer in finished.keys():
				item_names = [item.item_name + " x" + str(item.quantity) for item in finished[answer].items]
				print
				print "Notifying customer of order",answer,"completion:",item_names #orders[answer]
				print
				customer_msg = "Order " + str(answer) + " is ready for pickup: " + str(item_names)
				customer_pub.publish(std_msgs.msg.String(customer_msg))
				finished.pop(answer)

def printPending():
	print "Completed Order Ids:"
	for x in finished.keys():
		item_names = [item.item_name + " x" + str(item.quantity) for item in finished[x].items]
		print x,"\t",item_names
	print
	print "Pending Order Ids:"
	for x in orders.keys():
		item_names = [item.item_name + " x" + str(item.quantity) for item in orders[x].items]
		print x,"\t",item_names
	print

def cancelId(response):
	num = int(response.data)
	orders.pop(num)
	print "Order",num,"cancelled by customer"


def printUpdate(response):
	global active
	print "UPDATE", response.data
	if response.data == "DONE":
		if active in orders.keys():
			finished[active] = orders[active]
			orders.pop(active)
		active = 0
		print "Type 'list' to see all jobs, 'quit' to quit"
		print "Type an id of a pending job to dispatch Fetch"
		print "Type an id of a completed job to notify customer"
	
def main():
	global active
	active = 0
	rospy.init_node('command_line_dispatch')
	rospy.Subscriber('handle_order', acciobot_main.msg.Order, addOrder)
	rospy.Subscriber('print_update', std_msgs.msg.String, printUpdate)
	rospy.Subscriber('cancel_order', std_msgs.msg.String, cancelId)
	dispatch_pub = rospy.Publisher('dispatch_order', acciobot_main.msg.Order, latch=True, queue_size=10)
	customer_pub = rospy.Publisher('customer_update', std_msgs.msg.String, queue_size=10)
	rospy.sleep(1)
	printOrder(dispatch_pub, customer_pub)
	rospy.spin()

if __name__ == "__main__":
	main()
