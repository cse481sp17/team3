#!/usr/bin/env python

import os.path
import pickle

import rospy

import std_msgs.msg

import acciobot_main.msg

ITEM_FILE = '/home/team3/catkin_ws/src/cse481c/acciobot_main/items.pickle'

def wait_for_time():
	while rospy.Time().now().to_sec() == 0:
		pass

def create_default_items():
	stock = acciobot_main.msg.ItemStock()

	stock.header = std_msgs.msg.Header()
	stock.header.stamp = rospy.Time.now()

	first_item = acciobot_main.msg.Item()
	first_item.item_name = "Cheezits"
	first_item.item_id = 1
	first_item.item_type = "sunleft"
	first_item.quantity = 4
	first_item.feducial_id = 14

	second_item = acciobot_main.msg.Item()
	second_item.item_name = "Pasta"
	second_item.item_id = 2
	second_item.item_type = "front"
	second_item.quantity = 6
	second_item.feducial_id = 11

	third_item = acciobot_main.msg.Item()
	third_item.item_name = "Hairdye"
	third_item.item_id = 3
	third_item.item_type = "front"
	third_item.quantity = 1
	third_item.feducial_id = 0

	fourth_item = acciobot_main.msg.Item()
	fourth_item.item_name = "Cereal"
	fourth_item.item_id = 4
	fourth_item.item_type = "sunleft"
	fourth_item.quantity = 3
	fourth_item.feducial_id = 15

	stock.items.append(first_item)
	stock.items.append(second_item)
	stock.items.append(third_item)
	stock.items.append(fourth_item)
	return stock

class ItemStockWrapper(object):
	def __init__(self, stocked_items):
		self.stocked_items = stocked_items

	def update_items(self, item_delta):
		for i, curr_item in enumerate(item_delta.items):
			if item_delta.items[i].quantity != 0:
				print("Updating items:", curr_item, item_delta.items[i].quantity)
			for j, stocked_item in enumerate(self.stocked_items.items):
				if stocked_item.item_id == curr_item.item_id:
					print("Updated the item quantity! YAAS!")
					self.stocked_items.items[j].quantity += item_delta.items[i].quantity
					break

def main():
	rospy.init_node('item_publisher')
	wait_for_time()

	if os.path.isfile(ITEM_FILE):
		items = None
		with open(ITEM_FILE, 'r') as f:
			items = pickle.load(f)
	else:
		print("ih")
		items = create_default_items()

	stock_wrapper = ItemStockWrapper(items)

	item_pub = rospy.Publisher('available_items/', acciobot_main.msg.ItemStock, queue_size=5, latch=True)
	item_sub = rospy.Subscriber('update_items/', acciobot_main.msg.ItemStock, stock_wrapper.update_items)

	rospy.sleep(0.5)

	while not rospy.is_shutdown():
		items.header = std_msgs.msg.Header()
		items.header.stamp = rospy.Time.now()

		item_pub.publish(items)
		rospy.sleep(0.5)

	# TODO(emersonn): Maybe hook a subscriber to the topic too for new items added? Also save the file?

if __name__ == "__main__":
	main()
