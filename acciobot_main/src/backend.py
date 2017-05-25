#!/usr/bin/env python
import fetch_api
import rospy
import pickle
from util import Order, Station, Item
import OrderHandler
import StationInformation
import Navigator
from demonstration_program import *
import threading

#import srv.HandleOrder

# TO RUN:
#   start gazebo
#   roslaunch applications nav_rviz.launch (for now) TODO make our own launch file that starts nav
#   rosrun acciobot_main backend.py

PROGRAM_FILE = '/home/team3/catkin_ws/src/cse481c/acciobot_main/demonstration.pickle'

class RobotState(object):
    def __init__(self):
        self.current_order = None
        self.lock = threading.Lock()

    def dispatch_order(self, order_id):
        with self.lock:
            if self.current_order is None:
                self.current_order = order_id

    def finished_order(self):
        with self.lock:
            self.current_order = None

    def get_current_order(self):
        with self.lock:
            return self.current_order

    def order_callback(order_msg):
        dispatch_order(order_msg.order_id)

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
    robot_state = RobotState()

    # TODO subscribe here
    #rospy.Subscriber()
    
    #move_service = rospy.Service('handle_orders', HandleOrders, order_handler.order_callback)

    rospy.on_shutdown(navigator.stop)

    while True:
        if robot_state.get_current_order() is not None:
            order = order_handler.remove_order(robot_state.get_current_order())
            order.fulfill_order()
            navigator.move_to_posestamped(station_handler.get_cashier().location)
            robot_state.finished_order()
        rospy.sleep(1)


if __name__ == "__main__":
    main()
