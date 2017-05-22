#!/usr/bin/env python
import fetch_api
import rospy
import pickle
from util import Order, Station, Item
import OrderHandler
import StationInformation
import Navigator
from demonstration_program import *
#import srv.HandleOrder

# TO RUN:
#   start gazebo
#   roslaunch applications nav_rviz.launch (for now) TODO make our own launch file that starts nav
#   rosrun acciobot_main backend.py

PROGRAM_FILE = '/home/team3/catkin_ws/src/cse481c/acciobot_main/demonstration.pickle'

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def load_locations(location_file):
    with open(location_file) as f:
        return pickle.load(f)

def load_stations(station_file):
    with open(station_file) as f:
        return pickle.load(f)

def main():
    rospy.init_node('accio_backend')
    wait_for_time()
    navigator = Navigator.Navigator()
    station_handler = StationInformation.StationInformation(navigator)
    program_handler = ProgramHandler(PROGRAM_FILE)
    gripper = Gripper()
    arm = Arm()
    load_program(PROGRAM_FILE, gripper, arm)
    order_handler = OrderHandler.OrderHandler(station_handler, program_handler)
    # item_locations = load_locations(...)
    # station_locations = load_stations(...)

    #move_service = rospy.Service('handle_orders', HandleOrders, order_handler.order_callback)
    rospy.on_shutdown(navigator.stop)
    while True:
        if (not order_handler.order_queue.empty()):
            order = order_handler.order_queue.get()
            order.fulfill_order()
            # TODO CashierHandler, that goes to cashier and blocks until cashier is ready
            navigator.move_to_posestamped(station_handler.get_cashier().location)
        rospy.sleep(1)


if __name__ == "__main__":
    main()
