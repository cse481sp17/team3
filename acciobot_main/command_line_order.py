#!/usr/bin/env python

import rospy
from acciobot_main.srv import HandleOrder, HandleOrderRequest
from acciobot_main.msg import Item
import std_msgs.msg
import acciobot_main.msg

class Stock():
	def __init__(self):
		self.stock = {}

	def _callback(self, response):
		for item in response.items:
			self.stock[item.item_id] = item
	def _printStock(self):
		print "Items available for purchase:"
		print "ID\tItem\t\tAmount"
		for item in self.stock.keys():
			print item,"\t",self.stock[item].item_name,"\t",self.stock[item].quantity
	def _getItemName(self, itemId):
		return self.stock[itemId].item_name
	def _getItem(self, itemId):
		if itemId in self.stock.keys():
			return self.stock[itemId]
		else:
			return None

class Cart():

	def __init__(self):
		self.cart = {}

	def _add(self, itemId, num, stock):
		if num < 0: 
			print "You cannot add a negative amount of items"
			return
		if stock._getItem(itemId) is None:
			print "No items match that ID"
			return 
		if itemId not in self.cart.keys():
			self.cart[itemId] = 0
		self.cart[itemId] = self.cart[itemId] + num

	def _del(self, item):
		self.cart.pop(item)

	def _printContents(self,stock):
		print "Items in cart:"
		print "ID\tItem\t\tAmount"
		for x in self.cart.keys():
			print x,"\t",stock._getItemName(int(x)),"\t",self.cart[x]
		print

	def _contents(self, stock):
		items = []
		for x in self.cart.keys():
			it = stock._getItem(x)
			it.quantity = -1 * self.cart[x]
			items += [it]
		return items


def print_intro():
	print ("Grocery Ordering Service for UNIX-confident customers")
	print ("Commands:")
	print ("   list: List items in cart and items for purchase.")
	print ("   add <id> <count>: Add <count> number of <name> items to order.")
	print ("   del <id>: Remove <id> items from order.")
	print ("   done: Complete order and submit.")
	print ("   status <id>: Display status of order number <id>.")
	print ("   help: Show this list of commands")
	print ("   quit: Exit program.")

def get_info(cart, stock):
	answer = ""
	command = ""
	name = ""
	count = 0
	raw_answer = raw_input("> ")
	if raw_answer == "quit":
		quit()
	if raw_answer == "list":
		cart._printContents(stock)
		stock._printStock()
	elif raw_answer == "help":
		print_intro()
	elif raw_answer == "done":
		return True
	else:
		try:
			answer = raw_answer.split(" ")[0]
			name = int(raw_answer.split(" ")[1])
			if answer == "add":
				count = int(raw_answer.split(" ")[2])
				cart._add(name, count, stock)
			elif answer == "del":
				cart._del(name)
		except:
			print "unknown command"
	return False

def main():
	rospy.init_node('command_line_order')
	rospy.wait_for_service('acciobot_main/handle_order')
	send_order = rospy.ServiceProxy('acciobot_main/handle_order', HandleOrder)
	stock = Stock()
	rospy.Subscriber('available_items', acciobot_main.msg.ItemStock, stock._callback)
	order_pub = rospy.Publisher('update_items', acciobot_main.msg.ItemStock, queue_size=10)

	while True:
		print_intro()
		cart = Cart()
		done = False
		while not done:
			done = get_info(cart, stock)
		try:
			items = cart._contents(stock)
			stocker = acciobot_main.msg.ItemStock()
			stocker.items = items
			stocker.header = std_msgs.msg.Header()
#			print stocker
			order_pub.publish(stocker)
		#response = send_order(HandleOrderRequest.ORDER,items,quantity)
		#print response
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))


if __name__ == "__main__":
	main()
