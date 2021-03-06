#!/usr/bin/env python
import sys
import rospy
from acciobot_main.srv import HandleOrder, HandleOrderRequest
from acciobot_main.msg import Item
import std_msgs.msg
import acciobot_main.msg

finished = {}

def finishedCallback(response):
	global finished
	key = str(response.data)[6:].split(" ")[0]
	num = int(key)
	finished[num] = response
	print response.data

class Stock():
	def __init__(self):
		self.stock = {}
		self.length = 0

	def _callback(self, response):
		self.length = 0
		for item in response.items:
			self.stock[item.item_id] = item
			if len(item.item_name) > self.length: 
				self.length = len(item.item_name)
	def _printStock(self):
		print "Items available for purchase:"
		pad = " " * (self.length - 4)
		print "ID\tItem"+pad+"\tAmount"
		for item in self.stock.keys():
			padding = " " * (self.length - len(self.stock[item].item_name))
			print item,"\t",self.stock[item].item_name+padding,"\t",self.stock[item].quantity
	def _getItemName(self, itemId):
		return self.stock[itemId].item_name
	def _getItem(self, itemId):
		if itemId in self.stock.keys():
			return self.stock[itemId]
		else:
			return None
	def _getLength(self):
		return self.length

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
		pad = " " * (stock._getLength() - 4)
		print "ID\tItem\t\tAmount"
		for x in self.cart.keys():
			padding = " " * (stock._getLength() - len(stock._getItemName(int(x))))
			print x,"\t",stock._getItemName(int(x)) + padding,"\t",self.cart[x]
		print

	def _contents(self, stock):
		items = []
		for x in self.cart.keys():
			it = stock._getItem(x)
			it.quantity = self.cart[x]
			items += [it]
		print items
		return items


def print_intro():
	print ("Grocery Ordering Service for UNIX-confident customers")
	print ("Commands:")
	print ("   list: List items in cart and items for purchase.")
	print ("   add <id> <count>: Add <count> number of <name> items to order.")
	print ("   del <id>: Remove <id> items from order.")
	print ("   done: Complete order and submit.")
	print ("   cancel <id>: Cancel order number <id>.")
	print ("   help: Show this list of commands")
	print ("   quit: Exit program.")

def get_info(cart, stock):
	global finished
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
			elif answer == "cancel":
				print "returning",int(raw_answer.split(" ")[1])
				return int(raw_answer.split(" ")[1])
			elif answer == "status":
				print finished[name]
		except:
			print finished
			e = sys.exc_info()[0]
			print e
			print "unknown command"
	return False

def main():
	global finished
	rospy.init_node('command_line_order')
	rospy.wait_for_service('acciobot_main/handle_order')
	send_order = rospy.ServiceProxy('acciobot_main/handle_order', HandleOrder)
	stock = Stock()
	rospy.Subscriber('available_items', acciobot_main.msg.ItemStock, stock._callback)
	rospy.Subscriber('customer_update', std_msgs.msg.String, finishedCallback)
	order_pub = rospy.Publisher('update_items', acciobot_main.msg.ItemStock, latch=True, queue_size=10)

	while True:
		print_intro()
		cart = Cart()
		done = False
		while not done:
			done = get_info(cart, stock)
		try:
			if done == True:
				items = cart._contents(stock)
				response = send_order(HandleOrderRequest.ORDER,items,0)
				print "Your order number is",response.id
				stocker = acciobot_main.msg.ItemStock()
				stocker.items = items
				stocker.header = std_msgs.msg.Header()
				for x in stocker.items:
					x.quantity = -1 * x.quantity
				order_pub.publish(stocker)
				finished[int(response.id)] = "SUBMITTED"
			else:
				response = str(send_order(HandleOrderRequest.CANCEL,items,done))
				print "Cancelled order"
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))


if __name__ == "__main__":
	main()
